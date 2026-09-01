/*
  IRP_ESP32_Controller.ino
  Cranfield MSc IRP embedded reference implementation

  Geometry-guided closed-loop contact-force control for UAV-based ultrasonic
  inspection using:
    - ESP32 (Arduino core 3.x)
    - VL53L5CX 8x8 ToF sensor (SparkFun VL53L5CX library)
    - BNO085 IMU (Adafruit BNO08x library)
    - analogue force sensor with conditioned 0-3.3 V output
    - position-feedback two-wire linear actuator through an H-bridge

  Loop rates:
    Force/actuator loop: 50 Hz
    Geometry/RANSAC loop: 15 Hz

  IMPORTANT SAFETY NOTE:
  The ADC calibration constants below are safe compile-time examples only.
  The actuator is disabled until CALIBRATION_VERIFIED is deliberately set true
  after bench calibration. This avoids presenting unmeasured calibration as
  validated hardware data.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <Adafruit_BNO08x.h>

// ---------------------------- Hardware pins ----------------------------
static const int PIN_I2C_SDA = 21;
static const int PIN_I2C_SCL = 22;
static const int PIN_FORCE_ADC = 34;      // input-only ADC pin on classic ESP32
static const int PIN_POSITION_ADC = 35;   // input-only ADC pin
static const int PIN_HBRIDGE_IN1 = 25;
static const int PIN_HBRIDGE_IN2 = 26;
static const int PIN_ESTOP = 27;          // active LOW hardware interlock

// ----------------------- Calibration interlock -------------------------
// Set true only after measuring and updating the two-point calibrations.
static const bool CALIBRATION_VERIFIED = false;

// Force sensor two-point calibration (replace with measured bench values).
static const float FORCE_CAL_N0 = 0.0f;
static const int   FORCE_CAL_ADC0 = 620;
static const float FORCE_CAL_N1 = 10.0f;
static const int   FORCE_CAL_ADC1 = 3100;

// Position sensor two-point calibration for actuator displacement.
// Local control uses only 0-10 mm even if the actuator has a longer stroke.
static const float POS_CAL_M0 = 0.000f;
static const int   POS_CAL_ADC0 = 400;
static const float POS_CAL_M1 = 0.050f;
static const int   POS_CAL_ADC1 = 3600;

// ------------------------ Control requirements -------------------------
static const float F_REF_N = 5.5f;
static const float F_TOL_LOW_N = 5.0f;
static const float F_TOL_HIGH_N = 6.0f;
static const float FORCE_HARD_MAX_N = 6.0f;
static const float CONTACT_PRESENT_N = 0.5f;

static const float X_MIN_M = 0.000f;
static const float X_MAX_M = 0.010f;
static const float X_RATE_MPS = 0.010f;      // 10 mm/s local rate limit

static const uint32_t FORCE_PERIOD_US = 20000;  // 50 Hz
static const uint32_t GEOM_PERIOD_US = 66667;   // ~15 Hz
static const float DT_FORCE = 0.020f;

// Assumed simulation plant family used only for feedforward/scheduling.
static const float K_PLANT_MIN = 780.0f;     // N/m
static const float K_PLANT_MAX = 900.0f;     // N/m

// Force PID gains in displacement-command form.
struct PIDGains { float kp, ki, kd; };
static const PIDGains GAINS_FIXED    = {1.55e-3f, 1.05e-2f, 6.0e-5f};
static const PIDGains GAINS_HIGH     = {2.50e-3f, 1.85e-2f, 1.0e-4f};
static const PIDGains GAINS_FALLBACK = {1.20e-3f, 8.00e-3f, 4.0e-5f};

static const float DERIVATIVE_FC_HZ = 8.0f;
static const float FORCE_FILTER_FC_HZ = 10.0f;

// Inner actuator position loop. Output is signed PWM [-255, 255].
static const float POS_KP_PWM_PER_M = 18000.0f;
static const float POS_DEADBAND_M = 0.00020f;
static const int PWM_MAX = 200; // reserve margin below full command

// -------------------------- RANSAC settings ----------------------------
static const int ROI_N = 36;              // central 6x6 of 8x8
static const int RANSAC_ITERS = 150;
static const float INLIER_THRESH_M = 0.018f;
static const float Q_RESIDUAL_SCALE_M = 0.012f;
static const float Q_ON = 0.70f;
static const float Q_OFF = 0.55f;
static const int Q_ON_PERSISTENCE = 3;
static const float ALIGN_ACQUIRE_DEG = 5.0f;
static const float ALIGN_RETRACT_DEG = 10.0f;

// VL53L5CX approximate optical model.
// 65 deg diagonal FoV -> approximately 48.4 deg on each square axis.
static const float AXIS_FOV_DEG = 48.4f;
static const float RANGE_MIN_M = 0.10f;
static const float RANGE_MAX_M = 2.00f;

// -------------------------- Sensor objects -----------------------------
SparkFun_VL53L5CX tof;
VL53L5CX_ResultsData tofData;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t bnoValue;

struct Vec3 { float x, y, z; };
struct Quat { float w, x, y, z; };

Vec3 pointsWorld[ROI_N];
bool pointValid[ROI_N];
int nValidPoints = 0;

Quat qWorldFromSensor = {1,0,0,0};
Vec3 estimatedNormal = {0,0,1};
float normalAngleDeg = 0.0f;
float ransacResidualM = 1.0f;
float ransacInlierRatio = 0.0f;
float qR = 0.0f;
float rhoG = 0.0f;
bool geometryTrusted = false;
int qGoodCount = 0;

float forceFilteredN = 0.0f;
float forceIntegral = 0.0f;
float previousForceError = 0.0f;
float derivativeFiltered = 0.0f;
float xCommandM = 0.0f;
float xPositionM = 0.0f;
PIDGains activeGains = GAINS_FALLBACK;

uint32_t lastForceUs = 0;
uint32_t lastGeomUs = 0;

// ----------------------------- Utilities -------------------------------
static float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static float lerpf(float a, float b, float u) {
  return a + u * (b - a);
}

static float adcToForceN(int adc) {
  const float slope = (FORCE_CAL_N1 - FORCE_CAL_N0) /
                      float(FORCE_CAL_ADC1 - FORCE_CAL_ADC0);
  return FORCE_CAL_N0 + slope * float(adc - FORCE_CAL_ADC0);
}

static float adcToPositionM(int adc) {
  const float slope = (POS_CAL_M1 - POS_CAL_M0) /
                      float(POS_CAL_ADC1 - POS_CAL_ADC0);
  return POS_CAL_M0 + slope * float(adc - POS_CAL_ADC0);
}

static Vec3 quatRotate(const Quat &q, const Vec3 &v) {
  // v' = q * [0,v] * q^-1, expanded for efficiency.
  const float tx = 2.0f * (q.y * v.z - q.z * v.y);
  const float ty = 2.0f * (q.z * v.x - q.x * v.z);
  const float tz = 2.0f * (q.x * v.y - q.y * v.x);
  Vec3 out;
  out.x = v.x + q.w * tx + (q.y * tz - q.z * ty);
  out.y = v.y + q.w * ty + (q.z * tx - q.x * tz);
  out.z = v.z + q.w * tz + (q.x * ty - q.y * tx);
  return out;
}

static void normalizeVec(Vec3 &v) {
  const float n = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
  if (n > 1e-8f) { v.x/=n; v.y/=n; v.z/=n; }
}

static bool solve3x3(float A[3][3], float b[3], float x[3]) {
  // Gaussian elimination with partial pivoting.
  for (int k=0;k<3;k++) {
    int pivot = k;
    float best = fabsf(A[k][k]);
    for (int i=k+1;i<3;i++) {
      float v = fabsf(A[i][k]);
      if (v > best) { best=v; pivot=i; }
    }
    if (best < 1e-9f) return false;
    if (pivot != k) {
      for (int j=k;j<3;j++) { float tmp=A[k][j]; A[k][j]=A[pivot][j]; A[pivot][j]=tmp; }
      float tb=b[k]; b[k]=b[pivot]; b[pivot]=tb;
    }
    const float d=A[k][k];
    for (int j=k;j<3;j++) A[k][j]/=d;
    b[k]/=d;
    for (int i=0;i<3;i++) {
      if (i==k) continue;
      const float f=A[i][k];
      for (int j=k;j<3;j++) A[i][j]-=f*A[k][j];
      b[i]-=f*b[k];
    }
  }
  x[0]=b[0]; x[1]=b[1]; x[2]=b[2];
  return true;
}

static bool fitPlaneLeastSquares(const bool inliers[ROI_N], Vec3 &normal, float &rms) {
  // Fit z = a*x + b*y + c to inlier points.
  float Sxx=0,Sxy=0,Sx=0,Syy=0,Sy=0,S1=0;
  float Sxz=0,Syz=0,Sz=0;
  int n=0;
  for (int i=0;i<ROI_N;i++) {
    if (!pointValid[i] || !inliers[i]) continue;
    const float x=pointsWorld[i].x, y=pointsWorld[i].y, z=pointsWorld[i].z;
    Sxx+=x*x; Sxy+=x*y; Sx+=x; Syy+=y*y; Sy+=y; S1+=1.0f;
    Sxz+=x*z; Syz+=y*z; Sz+=z; n++;
  }
  if (n < 3) return false;
  float A[3][3]={{Sxx,Sxy,Sx},{Sxy,Syy,Sy},{Sx,Sy,S1}};
  float bvec[3]={Sxz,Syz,Sz};
  float sol[3];
  if (!solve3x3(A,bvec,sol)) return false;
  const float a=sol[0], b=sol[1], c=sol[2];
  normal = {-a,-b,1.0f}; normalizeVec(normal);
  if (normal.z < 0) { normal.x=-normal.x; normal.y=-normal.y; normal.z=-normal.z; }
  float ss=0; int count=0;
  for (int i=0;i<ROI_N;i++) {
    if (!pointValid[i] || !inliers[i]) continue;
    const float pred=a*pointsWorld[i].x+b*pointsWorld[i].y+c;
    const float e=pointsWorld[i].z-pred;
    ss+=e*e; count++;
  }
  rms=sqrtf(ss/float(count));
  return true;
}

static bool ransacPlane(Vec3 &normal, float &residual, float &inlierRatio) {
  if (nValidPoints < 8) return false;
  bool bestInliers[ROI_N]={false};
  int bestCount=-1;
  float bestRms=1e9f;

  for (int iter=0;iter<RANSAC_ITERS;iter++) {
    int ids[3];
    for (int k=0;k<3;k++) {
      while (true) {
        int idx = random(0,ROI_N);
        if (!pointValid[idx]) continue;
        bool duplicate=false;
        for (int j=0;j<k;j++) if (ids[j]==idx) duplicate=true;
        if (!duplicate) { ids[k]=idx; break; }
      }
    }
    const Vec3 p1=pointsWorld[ids[0]], p2=pointsWorld[ids[1]], p3=pointsWorld[ids[2]];
    Vec3 u={p2.x-p1.x,p2.y-p1.y,p2.z-p1.z};
    Vec3 v={p3.x-p1.x,p3.y-p1.y,p3.z-p1.z};
    Vec3 n={u.y*v.z-u.z*v.y, u.z*v.x-u.x*v.z, u.x*v.y-u.y*v.x};
    const float nn=sqrtf(n.x*n.x+n.y*n.y+n.z*n.z);
    if (nn<1e-8f) continue;
    n.x/=nn; n.y/=nn; n.z/=nn;
    const float d=-(n.x*p1.x+n.y*p1.y+n.z*p1.z);
    bool inliers[ROI_N]={false}; int count=0; float ss=0;
    for (int i=0;i<ROI_N;i++) {
      if (!pointValid[i]) continue;
      const float dist=fabsf(n.x*pointsWorld[i].x+n.y*pointsWorld[i].y+n.z*pointsWorld[i].z+d);
      if (dist<INLIER_THRESH_M) { inliers[i]=true; count++; ss+=dist*dist; }
    }
    if (count<3) continue;
    const float rms=sqrtf(ss/float(count));
    if (count>bestCount || (count==bestCount && rms<bestRms)) {
      bestCount=count; bestRms=rms;
      for (int i=0;i<ROI_N;i++) bestInliers[i]=inliers[i];
    }
  }
  if (bestCount<3) return false;
  if (!fitPlaneLeastSquares(bestInliers,normal,residual)) return false;
  inlierRatio=float(bestCount)/float(nValidPoints);
  return true;
}

static PIDGains scheduledGains(float rho) {
  rho=clampf(rho,0.0f,1.0f);
  PIDGains g;
  g.kp=lerpf(GAINS_FIXED.kp,GAINS_HIGH.kp,rho);
  g.ki=lerpf(GAINS_FIXED.ki,GAINS_HIGH.ki,rho);
  g.kd=lerpf(GAINS_FIXED.kd,GAINS_HIGH.kd,rho);
  return g;
}

static void setActuatorPWM(int signedPwm) {
  signedPwm=constrain(signedPwm,-PWM_MAX,PWM_MAX);
  if (!CALIBRATION_VERIFIED || digitalRead(PIN_ESTOP)==LOW) signedPwm=0;
  if (signedPwm>0) {
    analogWrite(PIN_HBRIDGE_IN1,signedPwm);
    analogWrite(PIN_HBRIDGE_IN2,0);
  } else if (signedPwm<0) {
    analogWrite(PIN_HBRIDGE_IN1,0);
    analogWrite(PIN_HBRIDGE_IN2,-signedPwm);
  } else {
    analogWrite(PIN_HBRIDGE_IN1,0);
    analogWrite(PIN_HBRIDGE_IN2,0);
  }
}

static void updateIMU() {
  while (bno08x.getSensorEvent(&bnoValue)) {
    if (bnoValue.sensorId == SH2_ROTATION_VECTOR) {
      qWorldFromSensor.w=bnoValue.un.rotationVector.real;
      qWorldFromSensor.x=bnoValue.un.rotationVector.i;
      qWorldFromSensor.y=bnoValue.un.rotationVector.j;
      qWorldFromSensor.z=bnoValue.un.rotationVector.k;
      const float qn=sqrtf(qWorldFromSensor.w*qWorldFromSensor.w + qWorldFromSensor.x*qWorldFromSensor.x +
                           qWorldFromSensor.y*qWorldFromSensor.y + qWorldFromSensor.z*qWorldFromSensor.z);
      if (qn>1e-6f) {
        qWorldFromSensor.w/=qn; qWorldFromSensor.x/=qn;
        qWorldFromSensor.y/=qn; qWorldFromSensor.z/=qn;
      }
    }
  }
}

static bool updateGeometry() {
  updateIMU();
  if (!tof.isDataReady()) return false;
  if (!tof.getRangingData(&tofData)) return false;

  nValidPoints=0;
  for (int i=0;i<ROI_N;i++) pointValid[i]=false;
  int outIdx=0;
  const float halfFov=0.5f*AXIS_FOV_DEG;
  for (int row=1;row<=6;row++) {
    for (int col=1;col<=6;col++) {
      const int idx=row*8+col;
      const float r=0.001f*float(tofData.distance_mm[idx]);
      const uint8_t status=tofData.target_status[idx];
      const bool validStatus=(status==5 || status==9);
      if (validStatus && r>=RANGE_MIN_M && r<=RANGE_MAX_M) {
        const float axDeg=-halfFov+(float(col)+0.5f)*(AXIS_FOV_DEG/8.0f);
        const float ayDeg=-halfFov+(float(row)+0.5f)*(AXIS_FOV_DEG/8.0f);
        const float tx=tanf(axDeg*DEG_TO_RAD);
        const float ty=tanf(ayDeg*DEG_TO_RAD);
        const float norm=sqrtf(1.0f+tx*tx+ty*ty);
        Vec3 pSensor={r*tx/norm,r*ty/norm,r/norm};
        pointsWorld[outIdx]=quatRotate(qWorldFromSensor,pSensor);
        pointValid[outIdx]=true;
        nValidPoints++;
      }
      outIdx++;
    }
  }

  Vec3 n; float residual,ratio;
  if (!ransacPlane(n,residual,ratio)) {
    qR=0; geometryTrusted=false; qGoodCount=0; return false;
  }
  estimatedNormal=n;
  ransacResidualM=residual;
  ransacInlierRatio=ratio;
  qR=ratio*expf(-sq(residual/Q_RESIDUAL_SCALE_M));

  // Probe nominal axis is +Z in the transformed local frame.
  normalAngleDeg=acosf(clampf(estimatedNormal.z,-1.0f,1.0f))*RAD_TO_DEG;
  const float rhoAngle=clampf(fabsf(normalAngleDeg)/30.0f,0.0f,1.0f);
  const float rhoComplex=clampf(residual/INLIER_THRESH_M,0.0f,1.0f);
  rhoG=clampf(0.75f*rhoAngle+0.25f*rhoComplex,0.0f,1.0f);

  if (geometryTrusted) {
    if (qR<Q_OFF) { geometryTrusted=false; qGoodCount=0; }
  } else {
    if (qR>=Q_ON) {
      qGoodCount++;
      if (qGoodCount>=Q_ON_PERSISTENCE) geometryTrusted=true;
    } else qGoodCount=0;
  }
  return true;
}

static void updateForceController() {
  const int rawF=analogRead(PIN_FORCE_ADC);
  const float forceN=adcToForceN(rawF);
  const float alphaF=(2.0f*PI*FORCE_FILTER_FC_HZ*DT_FORCE)/(1.0f+2.0f*PI*FORCE_FILTER_FC_HZ*DT_FORCE);
  forceFilteredN += alphaF*(forceN-forceFilteredN);
  xPositionM=clampf(adcToPositionM(analogRead(PIN_POSITION_ADC)),POS_CAL_M0,POS_CAL_M1);

  const bool aligned=fabsf(normalAngleDeg)<=ALIGN_ACQUIRE_DEG;
  if (geometryTrusted) activeGains=scheduledGains(rhoG);
  else activeGains=GAINS_FALLBACK;

  const float Knom=K_PLANT_MAX-(K_PLANT_MAX-K_PLANT_MIN)*rhoG;
  const float xFeedforward=F_REF_N/Knom;
  const float e=F_REF_N-forceFilteredN;
  const float de=(e-previousForceError)/DT_FORCE;
  const float alphaD=(2.0f*PI*DERIVATIVE_FC_HZ*DT_FORCE)/(1.0f+2.0f*PI*DERIVATIVE_FC_HZ*DT_FORCE);
  derivativeFiltered += alphaD*(de-derivativeFiltered);

  const float uUnsat=xFeedforward + activeGains.kp*e + activeGains.ki*forceIntegral + activeGains.kd*derivativeFiltered;
  float uSat=clampf(uUnsat,X_MIN_M,X_MAX_M);

  // Conditional integration anti-windup.
  const bool canIntegrate=(fabsf(uSat-uUnsat)<1e-7f) ||
                          (uSat>=X_MAX_M && e<0) ||
                          (uSat<=X_MIN_M && e>0);
  if (canIntegrate) forceIntegral += e*DT_FORCE;

  // Rate limit the displacement command.
  const float maxStep=X_RATE_MPS*DT_FORCE;
  xCommandM += clampf(uSat-xCommandM,-maxStep,maxStep);
  xCommandM=clampf(xCommandM,X_MIN_M,X_MAX_M);

  // Supervisory safety: retract on excessive force or large alignment error.
  if (forceFilteredN>=FORCE_HARD_MAX_N || fabsf(normalAngleDeg)>ALIGN_RETRACT_DEG || digitalRead(PIN_ESTOP)==LOW) {
    xCommandM=clampf(xPositionM-0.001f,X_MIN_M,X_MAX_M);
    forceIntegral=0.0f;
  }

  const float posErr=xCommandM-xPositionM;
  int pwm=0;
  if (fabsf(posErr)>POS_DEADBAND_M) pwm=int(POS_KP_PWM_PER_M*posErr);
  setActuatorPWM(pwm);

  previousForceError=e;

  const bool utAcquire=CALIBRATION_VERIFIED && geometryTrusted && aligned &&
                       forceFilteredN>=F_TOL_LOW_N && forceFilteredN<=F_TOL_HIGH_N;

  Serial.printf("F=%.3fN,x=%.4fm,xcmd=%.4fm,qR=%.3f,rho=%.3f,angle=%.2fdeg,valid=%d,UT=%d\n",
                forceFilteredN,xPositionM,xCommandM,qR,rhoG,normalAngleDeg,nValidPoints,utAcquire?1:0);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(PIN_HBRIDGE_IN1,OUTPUT);
  pinMode(PIN_HBRIDGE_IN2,OUTPUT);
  pinMode(PIN_ESTOP,INPUT_PULLUP);
  setActuatorPWM(0);

  analogReadResolution(12);
  Wire.begin(PIN_I2C_SDA,PIN_I2C_SCL,400000);
  randomSeed(esp_random());

  if (!tof.begin()) {
    Serial.println("ERROR: VL53L5CX not detected.");
    while (1) delay(1000);
  }
  tof.setResolution(8*8);
  tof.setRangingFrequency(15);
  tof.startRanging();

  if (!bno08x.begin_I2C()) {
    Serial.println("ERROR: BNO085 not detected.");
    while (1) delay(1000);
  }
  bno08x.enableReport(SH2_ROTATION_VECTOR,10000); // 100 Hz orientation output

  forceFilteredN=adcToForceN(analogRead(PIN_FORCE_ADC));
  xPositionM=adcToPositionM(analogRead(PIN_POSITION_ADC));
  xCommandM=clampf(xPositionM,X_MIN_M,X_MAX_M);
  lastForceUs=micros();
  lastGeomUs=micros();

  Serial.println("IRP controller started.");
  if (!CALIBRATION_VERIFIED) {
    Serial.println("ACTUATOR DISABLED: update measured calibration constants and set CALIBRATION_VERIFIED=true.");
  }
}

void loop() {
  const uint32_t now=micros();
  if ((uint32_t)(now-lastGeomUs)>=GEOM_PERIOD_US) {
    lastGeomUs+=GEOM_PERIOD_US;
    updateGeometry();
  }
  if ((uint32_t)(now-lastForceUs)>=FORCE_PERIOD_US) {
    lastForceUs+=FORCE_PERIOD_US;
    updateForceController();
  }
}
