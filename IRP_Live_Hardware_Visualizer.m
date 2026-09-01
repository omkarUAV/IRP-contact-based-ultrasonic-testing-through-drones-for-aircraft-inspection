%% IRP_Live_Hardware_Visualizer.m
% Live ESP32 + VL53L5CX + BNO085 + Force Sensor visualizer
%
% Expected ESP32 CSV packet (one line per control update):
% timestamp_us,force_N,ransac_normal_deg,inliers,plane_rmse_mm,bno085_angle_deg,pid_output_mm
%
% Example:
% 1534200,5.43,7.81,31,2.64,8.20,6.31
%
% IRP baseline:
%   Force reference: 5.5 N
%   Acceptable force band: 5.0 to 6.0 N
%   Force/PID update: 50 Hz
%   Geometry update: 15 Hz
%   VL53L5CX ROI: central 6x6 = 36 points
%   RANSAC: 150 iterations, 8 mm threshold, minimum 20 inliers
%
% IMPORTANT:
% UT quality below is only a LIVE QUALITY PROXY calculated from force,
% alignment and RANSAC confidence. It is NOT measured A-scan/SNR data.

clear; clc; close all;

%% --------------------------- USER SETTINGS -----------------------------
PORT = "COM5";               % CHANGE THIS to your ESP32 COM port
BAUD = 115200;               % must match ESP32 Serial.begin(...)
MAX_WINDOW_S = 20;           % rolling time window
DRAW_RATE_HZ = 20;           % figure refresh rate

FREF = 5.5;
FLOW = 5.0;
FHIGH = 6.0;

ROI_POINTS = 36;
MIN_INLIERS = 20;
RANSAC_ITER = 150;
RANSAC_THRESHOLD_MM = 8;

GOOD_QUALITY = 0.75;
POOR_QUALITY = 0.45;

%% ------------------------ CONNECT TO ESP32 ------------------------------
available = string(serialportlist("available"));

if ~any(available == PORT)
    fprintf("Available serial ports:\n");
    disp(available);
    error("Selected port %s is not available. Change PORT at the top of the script.", PORT);
end

s = serialport(PORT, BAUD);
configureTerminator(s, "LF");
flush(s);

fprintf("Connected to ESP32 on %s at %d baud.\n", PORT, BAUD);
fprintf("Waiting for CSV packets...\n");
fprintf("Format:\n");
fprintf("timestamp_us,force_N,ransac_normal_deg,inliers,plane_rmse_mm,bno085_angle_deg,pid_output_mm\n\n");

%% --------------------------- DATA BUFFERS -------------------------------
time_s = [];
force_N = [];
normal_deg = [];
inliers = [];
plane_rmse_mm = [];
imu_deg = [];
pid_mm = [];
quality = [];

t0_us = NaN;
lastDraw = tic;

%% --------------------------- CREATE FIGURE ------------------------------
fig = figure( ...
    'Color','w', ...
    'Name','IRP Live UAV Ultrasonic Inspection Visualizer', ...
    'NumberTitle','off', ...
    'Position',[40 40 1600 900], ...
    'CloseRequestFcn',@closeFigure);

% Scene
axScene = axes('Parent',fig,'Position',[0.05 0.12 0.43 0.76]);
hold(axScene,'on'); grid(axScene,'on'); box(axScene,'on');
xlim(axScene,[-2.2 2.2]); ylim(axScene,[-0.45 2.05]);
xlabel(axScene,'Aircraft Surface X [m]');
ylabel(axScene,'Height [m]');
title(axScene,{'LIVE Geometry-Guided UAV Contact Ultrasonic Inspection', ...
    'ESP32 + VL53L5CX + BNO085 + Force Sensor'},'FontWeight','bold');

Rsurf = 3.0;
xs = linspace(-2.0,2.0,500);
ys = 0.65 - (Rsurf - sqrt(max(Rsurf^2-xs.^2,0)));

patch(axScene,[xs fliplr(xs)], [ys -0.45*ones(size(xs))], ...
    [0.92 0.94 0.97], 'EdgeColor','none');
plot(axScene,xs,ys,'k','LineWidth',3);

% Use x=0 contact point for live hardware bench visualization.
xContact = 0;
yContact = 0.65;

hArm = plot(axScene,nan,nan,'k-','LineWidth',4);

hBody = rectangle(axScene,'Position',[-0.29 1.35 0.58 0.20], ...
    'Curvature',0.10,'FaceColor',[0.20 0.20 0.20], ...
    'EdgeColor','k','LineWidth',1.5);

hRotorL = rectangle(axScene,'Position',[-0.47 1.56 0.32 0.07], ...
    'Curvature',1,'FaceColor',[0.55 0.55 0.55],'EdgeColor','k');
hRotorR = rectangle(axScene,'Position',[0.15 1.56 0.32 0.07], ...
    'Curvature',1,'FaceColor',[0.55 0.55 0.55],'EdgeColor','k');

hSensor = rectangle(axScene,'Position',[-0.09 1.00 0.18 0.13], ...
    'Curvature',0.15,'FaceColor',[0.70 0.85 1.00], ...
    'EdgeColor',[0.15 0.40 0.70],'LineWidth',1.2);

hProbe = rectangle(axScene,'Position',[-0.06 0.66 0.12 0.20], ...
    'Curvature',0.10,'FaceColor',[1.00 0.78 0.18], ...
    'EdgeColor','k','LineWidth',1.2);

hContact = plot(axScene,xContact,yContact,'o','MarkerSize',10, ...
    'MarkerFaceColor',[1.00 0.75 0.00],'MarkerEdgeColor','k');

hRays = gobjects(6,1);
for r = 1:6
    hRays(r) = plot(axScene,nan,nan,'-','Color',[0.20 0.45 0.85], ...
        'LineWidth',1.1);
end

hState = text(axScene,-0.75,1.88,'WAITING FOR ESP32 DATA', ...
    'FontWeight','bold','FontSize',13,'Color',[0.35 0.35 0.35]);

hStatus = text(axScene,-2.05,1.78,'Waiting for first valid packet...', ...
    'VerticalAlignment','top','FontSize',10.5, ...
    'BackgroundColor','w','EdgeColor',[0.75 0.75 0.75],'Margin',8);

% Force
axF = axes('Parent',fig,'Position',[0.54 0.76 0.42 0.18]);
hold(axF,'on'); grid(axF,'on'); box(axF,'on');
hForce = plot(axF,nan,nan,'LineWidth',1.8);
yline(axF,FREF,'-','5.5 N target','LabelHorizontalAlignment','right');
yline(axF,FLOW,'--','5.0 N lower','LabelHorizontalAlignment','right');
yline(axF,FHIGH,'--','6.0 N upper','LabelHorizontalAlignment','right');
ylim(axF,[4.0 7.0]);
ylabel(axF,'Force [N]'); xlabel(axF,'Time [s]');
title(axF,'Live Contact Force');

% Angle
axA = axes('Parent',fig,'Position',[0.54 0.54 0.42 0.18]);
hold(axA,'on'); grid(axA,'on'); box(axA,'on');
hNormal = plot(axA,nan,nan,'LineWidth',1.4);
hIMU = plot(axA,nan,nan,'LineWidth',1.8);
yline(axA,30,'--','+30 deg','LabelHorizontalAlignment','right');
yline(axA,0,'-','0 deg','LabelHorizontalAlignment','right');
yline(axA,-30,'--','-30 deg','LabelHorizontalAlignment','right');
ylim(axA,[-35 35]);
ylabel(axA,'Angle [deg]'); xlabel(axA,'Time [s]');
title(axA,'RANSAC Surface Normal vs BNO085 Angle');
legend(axA,{'RANSAC normal','BNO085 angle'}, ...
    'Location','southoutside','Orientation','horizontal');

% RANSAC confidence
axG = axes('Parent',fig,'Position',[0.54 0.32 0.42 0.18]);
hold(axG,'on'); grid(axG,'on'); box(axG,'on');
hInlier = plot(axG,nan,nan,'LineWidth',1.8);
yline(axG,MIN_INLIERS/ROI_POINTS,'--','20/36 minimum', ...
    'LabelHorizontalAlignment','right');
yline(axG,0.85,'--','0.85 strong','LabelHorizontalAlignment','right');
ylim(axG,[0 1.02]);
ylabel(axG,'Inlier Ratio'); xlabel(axG,'Time [s]');
title(axG,sprintf('Live RANSAC Confidence: 6x6 ROI, %d iter, %.0f mm threshold', ...
    RANSAC_ITER,RANSAC_THRESHOLD_MM));

% Quality proxy
axQ = axes('Parent',fig,'Position',[0.54 0.10 0.42 0.18]);
hold(axQ,'on'); grid(axQ,'on'); box(axQ,'on');
hQuality = plot(axQ,nan,nan,'LineWidth',1.8);
yline(axQ,GOOD_QUALITY,'--','Good proxy','LabelHorizontalAlignment','right');
yline(axQ,POOR_QUALITY,'--','Poor proxy','LabelHorizontalAlignment','right');
ylim(axQ,[0 1.02]);
ylabel(axQ,'Quality Index'); xlabel(axQ,'Time [s]');
title(axQ,'Live UT Quality Proxy — not measured A-scan/SNR');

%% ---------------------------- LIVE LOOP --------------------------------
while isvalid(fig)
    if s.NumBytesAvailable <= 0
        pause(0.002);
        continue;
    end

    raw = strtrim(readline(s));

    % Ignore comments/header lines
    if strlength(raw) == 0 || startsWith(raw,"#")
        continue;
    end

    values = str2double(split(raw, ","));

    if numel(values) ~= 7 || any(isnan(values))
        fprintf("Ignored malformed packet: %s\n", raw);
        continue;
    end

    timestamp_us = values(1);
    F = values(2);
    nDeg = values(3);
    nIn = round(values(4));
    rmse = values(5);
    imu = values(6);
    pid = values(7);

    if isnan(t0_us)
        t0_us = timestamp_us;
    end

    tt = double(timestamp_us - t0_us)/1e6;

    % Basic range protection
    nIn = min(max(nIn,0),ROI_POINTS);
    ratio = nIn/ROI_POINTS;

    alignmentError = imu - nDeg;

    % Synthetic quality proxy only
    forcePenalty = exp(-(abs(F-FREF)/0.45)^2);
    alignmentPenalty = exp(-(abs(alignmentError)/4.5)^2);
    confidencePenalty = 0.45 + 0.55*ratio;
    q = min(max(forcePenalty*alignmentPenalty*confidencePenalty,0),1);

    % Append
    time_s(end+1) = tt;
    force_N(end+1) = F;
    normal_deg(end+1) = nDeg;
    inliers(end+1) = nIn;
    plane_rmse_mm(end+1) = rmse;
    imu_deg(end+1) = imu;
    pid_mm(end+1) = pid;
    quality(end+1) = q;

    % Keep rolling buffer compact
    keep = time_s >= max(0,tt-MAX_WINDOW_S-2);
    time_s = time_s(keep);
    force_N = force_N(keep);
    normal_deg = normal_deg(keep);
    inliers = inliers(keep);
    plane_rmse_mm = plane_rmse_mm(keep);
    imu_deg = imu_deg(keep);
    pid_mm = pid_mm(keep);
    quality = quality(keep);

    % Limit graphics refresh rate
    if toc(lastDraw) < 1/DRAW_RATE_HZ
        continue;
    end
    lastDraw = tic;

    % Update traces
    set(hForce,'XData',time_s,'YData',force_N);
    set(hNormal,'XData',time_s,'YData',normal_deg);
    set(hIMU,'XData',time_s,'YData',imu_deg);
    set(hInlier,'XData',time_s,'YData',inliers/ROI_POINTS);
    set(hQuality,'XData',time_s,'YData',quality);

    x1 = max(0,tt-MAX_WINDOW_S);
    x2 = max(MAX_WINDOW_S,tt);
    xlim(axF,[x1 x2]);
    xlim(axA,[x1 x2]);
    xlim(axG,[x1 x2]);
    xlim(axQ,[x1 x2]);

    % Live scene uses current measured BNO085 angle.
    theta = deg2rad(imu);
    nx = sin(theta);
    ny = cos(theta);

    probeCenter = [xContact + 0.10*nx, yContact + 0.10*ny];
    sensorCenter= [xContact + 0.42*nx, yContact + 0.42*ny];
    bodyCenter  = [xContact + 0.85*nx, yContact + 0.85*ny];

    set(hArm,'XData',[probeCenter(1) bodyCenter(1)], ...
             'YData',[probeCenter(2) bodyCenter(2)]);

    set(hBody,'Position',[bodyCenter(1)-0.29 bodyCenter(2)-0.10 0.58 0.20]);
    set(hRotorL,'Position',[bodyCenter(1)-0.47 bodyCenter(2)+0.11 0.32 0.07]);
    set(hRotorR,'Position',[bodyCenter(1)+0.15 bodyCenter(2)+0.11 0.32 0.07]);
    set(hSensor,'Position',[sensorCenter(1)-0.09 sensorCenter(2)-0.065 0.18 0.13]);
    set(hProbe,'Position',[probeCenter(1)-0.06 probeCenter(2)-0.10 0.12 0.20]);

    spread = linspace(-0.18,0.18,6);
    for r = 1:6
        xr = sensorCenter(1) + spread(r);
        xe = xContact + 0.55*spread(r);
        ye = 0.65 - (Rsurf - sqrt(max(Rsurf^2-xe^2,0)));
        set(hRays(r),'XData',[xr xe],'YData',[sensorCenter(2)-0.02 ye]);
    end

    % State logic
    if nIn < MIN_INLIERS
        stateText = 'RANSAC CONFIDENCE LOW - FALLBACK';
        stateColor = [0.75 0.15 0.05];
    elseif F < FLOW || F > FHIGH
        stateText = 'CONTACT FORCE OUTSIDE BAND';
        stateColor = [0.85 0.45 0.00];
    elseif abs(alignmentError) > 5
        stateText = 'ALIGNMENT ERROR HIGH';
        stateColor = [0.85 0.45 0.00];
    else
        stateText = 'CONTACT / GEOMETRY: GOOD';
        stateColor = [0.00 0.48 0.15];
    end

    set(hState,'String',stateText,'Color',stateColor);

    set(hStatus,'String',sprintf([ ...
        'Time: %.3f s\n' ...
        'Force: %.2f N\n' ...
        'Target / band: %.1f N / %.1f-%.1f N\n' ...
        'RANSAC normal: %+.2f deg\n' ...
        'BNO085 angle: %+.2f deg\n' ...
        'Alignment error: %+.2f deg\n' ...
        'RANSAC inliers: %d / %d\n' ...
        'Plane RMSE: %.2f mm\n' ...
        'PID output: %.2f mm\n' ...
        'Quality proxy: %.2f'], ...
        tt,F,FREF,FLOW,FHIGH,nDeg,imu,alignmentError,nIn,ROI_POINTS,rmse,pid,q));

    drawnow limitrate;
end

%% ---------------------------- CLEANUP ----------------------------------
clear s;

%% ------------------------- LOCAL FUNCTION ------------------------------
function closeFigure(src,~)
    delete(src);
end
