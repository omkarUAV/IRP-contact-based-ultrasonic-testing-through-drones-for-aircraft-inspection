%% IRP UAV-UT Thickness Error Simulation
% Closed-loop geometry-guided force control versus open-loop contact.
%
% This script simulates:
%   1. A varying aircraft-skin thickness profile
%   2. Contact-force and probe-angle disturbances
%   3. UT signal-quality variation and measurement dropout
%   4. Echo time-of-flight measurement
%   5. Thickness reconstruction and error metrics
%
% Outputs:
%   - MAE, RMSE, bias, standard deviation, maximum error
%   - Valid measurement rate
%   - Thickness/error/force/angle plots
%   - Workspace variables suitable for later Simulink integration
%
% Units:
%   - Internal distance: metres where explicitly stated
%   - Reported thickness/error: millimetres
%   - Echo time of flight: seconds
%
% Compatible with MATLAB R2020b or later.

clear; clc; close all;
rng(20);   % Repeatable simulation

%% 1. Simulation settings
Ts = 0.02;                     % Sample time [s]
Tend = 20;                     % Simulation duration [s]
time = (0:Ts:Tend).';
N = numel(time);

scanLength = 0.60;             % Scan distance [m]
scanPosition = linspace(0, scanLength, N).';

% Aluminium longitudinal-wave velocity.
% Replace this with the velocity obtained from your calibration block.
cL = 6320;                     % [m/s]

% Calibrated system delay: cable + probe + wear plate + electronics.
systemDelay = 0.85e-6;         % [s]

% Contact-force operating region
Fref = 5.0;                    % Desired force [N]
FminQuality = 3.0;             % Lower preferred force [N]
FmaxQuality = 6.0;             % Upper preferred force [N]

%% 2. Reference thickness profile
% Nominal 5 mm aircraft-skin coupon with two simulated material-loss regions.
nominalThickness_mm = 5.00;

defect1_mm = 0.70 .* exp(-0.5 .* ((scanPosition - 0.22) ./ 0.030).^2);
defect2_mm = 0.38 .* exp(-0.5 .* ((scanPosition - 0.44) ./ 0.050).^2);
manufacturingVariation_mm = 0.025 .* sin(2*pi*scanPosition/0.18);

referenceThickness_mm = nominalThickness_mm ...
                      - defect1_mm ...
                      - defect2_mm ...
                      + manufacturingVariation_mm;

%% 3. Contact-force simulation
% Open-loop contact: no active correction.
forceOpen_N = Fref ...
    + 1.25*sin(2*pi*0.33*time) ...
    + 0.70*sin(2*pi*1.30*time) ...
    + 0.32*randn(N,1);

% Add two transient contact disturbances.
forceOpen_N = forceOpen_N ...
    - 1.65*exp(-0.5*((time-7.0)/0.35).^2) ...
    + 1.40*exp(-0.5*((time-14.2)/0.28).^2);

% Closed-loop force response: discrete first-order controller model.
forceClosed_N = zeros(N,1);
forceClosed_N(1) = 4.2;

Kforce = 5.2;                  % Closed-loop correction gain
tauForce = 0.24;               % Force-loop response time [s]

for k = 2:N
    surfaceDisturbance = 0.50*sin(2*pi*0.55*time(k)) ...
                       + 0.18*sin(2*pi*1.70*time(k));

    controller = Kforce*(Fref - forceClosed_N(k-1));
    forceDerivative = (controller + surfaceDisturbance) / tauForce;

    forceClosed_N(k) = forceClosed_N(k-1) ...
                     + Ts*forceDerivative ...
                     + 0.045*randn;

    % Actuator/contact safety limits for this simulation.
    forceClosed_N(k) = min(max(forceClosed_N(k), 2.5), 6.5);
end

%% 4. Probe-normal angular error
% Open-loop angle error contains surface-curvature and UAV attitude components.
angleOpen_deg = 8.0*sin(2*pi*0.19*time) ...
              + 4.2*sin(2*pi*0.73*time) ...
              + 1.6*randn(N,1);

% Geometry-guided alignment using VL53L5CX + RANSAC + BNO085.
% Simulated as a filtered residual angle after closed-loop correction.
angleClosed_deg = zeros(N,1);
angleClosed_deg(1) = angleOpen_deg(1);

tauAngle = 0.32;               % Geometry/attitude-loop time constant [s]
for k = 2:N
    desiredResidual = 0.22*angleOpen_deg(k);
    angleClosed_deg(k) = angleClosed_deg(k-1) ...
        + Ts/tauAngle*(desiredResidual - angleClosed_deg(k-1)) ...
        + 0.12*randn;
end

%% 5. Equivalent couplant-layer variation
% The layer is compressed by force. This is an equivalent timing model,
% not a detailed acoustic multilayer model.
couplantNominal_mm = 0.060;

couplantOpen_mm = couplantNominal_mm ...
    + 0.020*(Fref - forceOpen_N) ...
    + 0.009*randn(N,1);

couplantClosed_mm = couplantNominal_mm ...
    + 0.020*(Fref - forceClosed_N) ...
    + 0.003*randn(N,1);

couplantOpen_mm = max(couplantOpen_mm, 0.010);
couplantClosed_mm = max(couplantClosed_mm, 0.010);

%% 6. UT signal-quality model
% Quality is reduced by force deviation, angular error and couplant variation.
qualityOpen = exp(-((forceOpen_N-Fref)/1.55).^2) ...
            .* exp(-(abs(angleOpen_deg)/9.0).^2) ...
            .* exp(-((couplantOpen_mm-couplantNominal_mm)/0.035).^2);

qualityClosed = exp(-((forceClosed_N-Fref)/1.55).^2) ...
              .* exp(-(abs(angleClosed_deg)/9.0).^2) ...
              .* exp(-((couplantClosed_mm-couplantNominal_mm)/0.035).^2);

qualityOpen = min(max(qualityOpen,0),1);
qualityClosed = min(max(qualityClosed,0),1);

% Approximate SNR relation for plotting.
snrOpen_dB = 8 + 28*qualityOpen + 1.2*randn(N,1);
snrClosed_dB = 8 + 28*qualityClosed + 0.7*randn(N,1);

%% 7. Thickness-error mechanism
% The model below converts force, angle and couplant instability into
% thickness bias and random timing uncertainty.

forceBiasOpen_mm = 0.020*(forceOpen_N-Fref).^2 ...
                 + 0.014*max(FminQuality-forceOpen_N,0).^2 ...
                 + 0.014*max(forceOpen_N-FmaxQuality,0).^2;

forceBiasClosed_mm = 0.020*(forceClosed_N-Fref).^2 ...
                   + 0.014*max(FminQuality-forceClosed_N,0).^2 ...
                   + 0.014*max(forceClosed_N-FmaxQuality,0).^2;

angleBiasOpen_mm = 0.0018*angleOpen_deg.^2;
angleBiasClosed_mm = 0.0018*angleClosed_deg.^2;

couplantBiasOpen_mm = 1.50*(couplantOpen_mm-couplantNominal_mm);
couplantBiasClosed_mm = 1.50*(couplantClosed_mm-couplantNominal_mm);

% Random thickness noise increases when UT quality falls.
noiseStdOpen_mm = 0.018 + 0.110*(1-qualityOpen);
noiseStdClosed_mm = 0.012 + 0.060*(1-qualityClosed);

measurementNoiseOpen_mm = noiseStdOpen_mm .* randn(N,1);
measurementNoiseClosed_mm = noiseStdClosed_mm .* randn(N,1);

% Signed measurement errors.
trueErrorOpen_mm = forceBiasOpen_mm ...
                 + angleBiasOpen_mm ...
                 + couplantBiasOpen_mm ...
                 + measurementNoiseOpen_mm;

trueErrorClosed_mm = forceBiasClosed_mm ...
                   + angleBiasClosed_mm ...
                   + couplantBiasClosed_mm ...
                   + measurementNoiseClosed_mm;

% Simulated thickness sensed by the UT instrument.
simulatedUTThicknessOpen_mm = referenceThickness_mm + trueErrorOpen_mm;
simulatedUTThicknessClosed_mm = referenceThickness_mm + trueErrorClosed_mm;

%% 8. Convert simulated thickness into echo time of flight
tofOpen_s = systemDelay ...
          + 2*(simulatedUTThicknessOpen_mm/1000)/cL;

tofClosed_s = systemDelay ...
            + 2*(simulatedUTThicknessClosed_mm/1000)/cL;

% Add electronic timing jitter, scaled by signal quality.
timingJitterOpen_s = (1.5e-9 + 8.0e-9*(1-qualityOpen)) .* randn(N,1);
timingJitterClosed_s = (1.0e-9 + 4.0e-9*(1-qualityClosed)) .* randn(N,1);

tofOpen_s = tofOpen_s + timingJitterOpen_s;
tofClosed_s = tofClosed_s + timingJitterClosed_s;

%% 9. Measurement dropout
% A valid back-wall echo becomes less likely at poor signal quality.
dropProbabilityOpen = min(0.55, 0.45*(1-qualityOpen).^1.7);
dropProbabilityClosed = min(0.30, 0.22*(1-qualityClosed).^1.7);

validOpen = rand(N,1) > dropProbabilityOpen;
validClosed = rand(N,1) > dropProbabilityClosed;

% Also reject very poor SNR.
validOpen = validOpen & snrOpen_dB >= 12;
validClosed = validClosed & snrClosed_dB >= 12;

tofOpen_s(~validOpen) = NaN;
tofClosed_s(~validClosed) = NaN;

%% 10. Reconstruct thickness from ultrasonic ToF
measuredThicknessOpen_mm = 1000 * 0.5*cL*(tofOpen_s-systemDelay);
measuredThicknessClosed_mm = 1000 * 0.5*cL*(tofClosed_s-systemDelay);

errorOpen_mm = measuredThicknessOpen_mm-referenceThickness_mm;
errorClosed_mm = measuredThicknessClosed_mm-referenceThickness_mm;

percentErrorOpen = 100*abs(errorOpen_mm)./referenceThickness_mm;
percentErrorClosed = 100*abs(errorClosed_mm)./referenceThickness_mm;

%% 11. Performance metrics
metricsOpen = calculateMetrics(measuredThicknessOpen_mm, referenceThickness_mm);
metricsClosed = calculateMetrics(measuredThicknessClosed_mm, referenceThickness_mm);

fprintf('\n============================================================\n');
fprintf(' IRP UAV-UT THICKNESS-ERROR SIMULATION\n');
fprintf('============================================================\n');
printMetrics('OPEN LOOP', metricsOpen);
printMetrics('CLOSED LOOP', metricsClosed);

maeImprovement = 100*(metricsOpen.MAE_mm-metricsClosed.MAE_mm) ...
                    /metricsOpen.MAE_mm;
rmseImprovement = 100*(metricsOpen.RMSE_mm-metricsClosed.RMSE_mm) ...
                     /metricsOpen.RMSE_mm;

fprintf('\nClosed-loop MAE improvement  : %.2f %%\n', maeImprovement);
fprintf('Closed-loop RMSE improvement : %.2f %%\n', rmseImprovement);
fprintf('============================================================\n\n');

%% 12. Main results figure
figure('Name','IRP UT Thickness Error Simulation',...
       'Color','w','Position',[100 80 1250 820]);

tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

nexttile([1 2]);
plot(scanPosition, referenceThickness_mm,'k','LineWidth',2.2); hold on;
plot(scanPosition, measuredThicknessOpen_mm,'LineWidth',1.0);
plot(scanPosition, measuredThicknessClosed_mm,'LineWidth',1.3);
grid on;
xlabel('Scan position [m]');
ylabel('Thickness [mm]');
title('Reference and measured thickness');
legend('Reference','Open loop','Closed loop','Location','best');

nexttile;
plot(scanPosition,errorOpen_mm,'LineWidth',1.0); hold on;
yline(0,'k--');
grid on;
xlabel('Scan position [m]');
ylabel('Signed error [mm]');
title(sprintf('Open loop: MAE = %.3f mm',metricsOpen.MAE_mm));

nexttile;
plot(scanPosition,errorClosed_mm,'LineWidth',1.0); hold on;
yline(0,'k--');
grid on;
xlabel('Scan position [m]');
ylabel('Signed error [mm]');
title(sprintf('Closed loop: MAE = %.3f mm',metricsClosed.MAE_mm));

nexttile;
plot(time,forceOpen_N,'LineWidth',0.9); hold on;
plot(time,forceClosed_N,'LineWidth',1.1);
yline(Fref,'k--','5 N reference');
yline(FminQuality,'k:');
yline(FmaxQuality,'k:');
grid on;
xlabel('Time [s]');
ylabel('Contact force [N]');
title('Contact-force response');
legend('Open loop','Closed loop','Location','best');

nexttile;
plot(time,angleOpen_deg,'LineWidth',0.9); hold on;
plot(time,angleClosed_deg,'LineWidth',1.1);
yline(0,'k--');
grid on;
xlabel('Time [s]');
ylabel('Normal-angle error [deg]');
title('Probe alignment error');
legend('Open loop','Closed loop','Location','best');

%% 13. Error-analysis figure
figure('Name','IRP UT Error Analysis',...
       'Color','w','Position',[140 100 1250 780]);

tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile;
scatter(forceOpen_N(validOpen),abs(errorOpen_mm(validOpen)),10,...
    'filled','MarkerFaceAlpha',0.30); hold on;
scatter(forceClosed_N(validClosed),abs(errorClosed_mm(validClosed)),10,...
    'filled','MarkerFaceAlpha',0.30);
grid on;
xlabel('Contact force [N]');
ylabel('Absolute thickness error [mm]');
title('Error versus force');
legend('Open loop','Closed loop','Location','best');

nexttile;
scatter(abs(angleOpen_deg(validOpen)),abs(errorOpen_mm(validOpen)),10,...
    'filled','MarkerFaceAlpha',0.30); hold on;
scatter(abs(angleClosed_deg(validClosed)),abs(errorClosed_mm(validClosed)),10,...
    'filled','MarkerFaceAlpha',0.30);
grid on;
xlabel('|Probe normal-angle error| [deg]');
ylabel('Absolute thickness error [mm]');
title('Error versus alignment');
legend('Open loop','Closed loop','Location','best');

nexttile;
histogram(errorOpen_mm(validOpen),35,'Normalization','probability',...
    'FaceAlpha',0.55); hold on;
histogram(errorClosed_mm(validClosed),35,'Normalization','probability',...
    'FaceAlpha',0.55);
grid on;
xlabel('Signed thickness error [mm]');
ylabel('Probability');
title('Error distribution');
legend('Open loop','Closed loop','Location','best');

nexttile;
plot(scanPosition,snrOpen_dB,'LineWidth',0.9); hold on;
plot(scanPosition,snrClosed_dB,'LineWidth',1.1);
yline(12,'k--','Validity threshold');
grid on;
xlabel('Scan position [m]');
ylabel('Estimated SNR [dB]');
title('UT signal quality');
legend('Open loop','Closed loop','Location','best');

%% 14. Bland-Altman comparison
figure('Name','Bland-Altman Thickness Agreement',...
       'Color','w','Position',[180 120 1200 480]);

tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

nexttile;
blandAltmanPlot(referenceThickness_mm, measuredThicknessOpen_mm, ...
    'Open-loop agreement');

nexttile;
blandAltmanPlot(referenceThickness_mm, measuredThicknessClosed_mm, ...
    'Closed-loop agreement');

%% 15. Export variables for Simulink or further analysis
simulationResults = struct;

simulationResults.time_s = time;
simulationResults.scanPosition_m = scanPosition;
simulationResults.referenceThickness_mm = referenceThickness_mm;

simulationResults.openLoop.force_N = forceOpen_N;
simulationResults.openLoop.angle_deg = angleOpen_deg;
simulationResults.openLoop.couplant_mm = couplantOpen_mm;
simulationResults.openLoop.quality = qualityOpen;
simulationResults.openLoop.SNR_dB = snrOpen_dB;
simulationResults.openLoop.tof_s = tofOpen_s;
simulationResults.openLoop.thickness_mm = measuredThicknessOpen_mm;
simulationResults.openLoop.error_mm = errorOpen_mm;
simulationResults.openLoop.percentError = percentErrorOpen;
simulationResults.openLoop.metrics = metricsOpen;

simulationResults.closedLoop.force_N = forceClosed_N;
simulationResults.closedLoop.angle_deg = angleClosed_deg;
simulationResults.closedLoop.couplant_mm = couplantClosed_mm;
simulationResults.closedLoop.quality = qualityClosed;
simulationResults.closedLoop.SNR_dB = snrClosed_dB;
simulationResults.closedLoop.tof_s = tofClosed_s;
simulationResults.closedLoop.thickness_mm = measuredThicknessClosed_mm;
simulationResults.closedLoop.error_mm = errorClosed_mm;
simulationResults.closedLoop.percentError = percentErrorClosed;
simulationResults.closedLoop.metrics = metricsClosed;

simulationResults.parameters.Ts_s = Ts;
simulationResults.parameters.waveVelocity_mps = cL;
simulationResults.parameters.systemDelay_s = systemDelay;
simulationResults.parameters.forceReference_N = Fref;

% Timeseries variables can be connected directly to Simulink From Workspace blocks.
tofOpen_ts = timeseries(tofOpen_s,time);
tofClosed_ts = timeseries(tofClosed_s,time);
referenceThickness_ts = timeseries(referenceThickness_mm/1000,time);
forceOpen_ts = timeseries(forceOpen_N,time);
forceClosed_ts = timeseries(forceClosed_N,time);
angleOpen_ts = timeseries(angleOpen_deg,time);
angleClosed_ts = timeseries(angleClosed_deg,time);

save('IRP_UT_Thickness_Error_Results.mat',...
    'simulationResults',...
    'tofOpen_ts','tofClosed_ts','referenceThickness_ts',...
    'forceOpen_ts','forceClosed_ts','angleOpen_ts','angleClosed_ts');

fprintf('Saved: IRP_UT_Thickness_Error_Results.mat\n');

%% Local functions
function metrics = calculateMetrics(measured_mm,reference_mm)
    valid = isfinite(measured_mm) & isfinite(reference_mm) & reference_mm > 0;

    measured = measured_mm(valid);
    reference = reference_mm(valid);
    err = measured-reference;

    metrics.ValidSamples = nnz(valid);
    metrics.TotalSamples = numel(reference_mm);
    metrics.ValidRate_percent = 100*metrics.ValidSamples/metrics.TotalSamples;

    if isempty(err)
        metrics.MAE_mm = NaN;
        metrics.RMSE_mm = NaN;
        metrics.Bias_mm = NaN;
        metrics.Std_mm = NaN;
        metrics.MaxAbsoluteError_mm = NaN;
        metrics.MeanAbsolutePercentError = NaN;
        return
    end

    metrics.MAE_mm = mean(abs(err));
    metrics.RMSE_mm = sqrt(mean(err.^2));
    metrics.Bias_mm = mean(err);
    metrics.Std_mm = std(err);
    metrics.MaxAbsoluteError_mm = max(abs(err));
    metrics.MeanAbsolutePercentError = ...
        mean(100*abs(err)./reference);
end

function printMetrics(label,metrics)
    fprintf('\n%s\n',label);
    fprintf('  Valid measurement rate : %7.2f %%\n',...
        metrics.ValidRate_percent);
    fprintf('  MAE                    : %7.4f mm\n',...
        metrics.MAE_mm);
    fprintf('  RMSE                   : %7.4f mm\n',...
        metrics.RMSE_mm);
    fprintf('  Bias                   : %7.4f mm\n',...
        metrics.Bias_mm);
    fprintf('  Standard deviation     : %7.4f mm\n',...
        metrics.Std_mm);
    fprintf('  Maximum absolute error : %7.4f mm\n',...
        metrics.MaxAbsoluteError_mm);
    fprintf('  Mean absolute %% error  : %7.3f %%\n',...
        metrics.MeanAbsolutePercentError);
end

function blandAltmanPlot(reference_mm,measured_mm,plotTitle)
    valid = isfinite(reference_mm) & isfinite(measured_mm);
    reference = reference_mm(valid);
    measured = measured_mm(valid);

    meanThickness = 0.5*(reference+measured);
    difference = measured-reference;

    bias = mean(difference);
    sd = std(difference);
    upperLimit = bias+1.96*sd;
    lowerLimit = bias-1.96*sd;

    scatter(meanThickness,difference,12,'filled',...
        'MarkerFaceAlpha',0.35); hold on;
    yline(bias,'k-','Bias','LineWidth',1.4);
    yline(upperLimit,'k--','+1.96 SD','LineWidth',1.1);
    yline(lowerLimit,'k--','-1.96 SD','LineWidth',1.1);
    grid on;
    xlabel('Mean of reference and measured thickness [mm]');
    ylabel('Measured - reference [mm]');
    title(plotTitle);
end
