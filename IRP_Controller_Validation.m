%% IRP_Controller_Validation.m
% Geometry-guided fixed-vs-scheduled PID validation for UAV contact UT.
% Full executable MATLAB code; no pseudocode.
%
% Model scope:
%   - 50 Hz force loop
%   - 15 Hz geometry/scheduling update
%   - Assumed first-order contact-force plant family
%       tau*dF/dt + F = Kplant*x_a + d_F
%   - Rate-limited actuator displacement state x_a
%   - Fixed and geometry-scheduled PID controllers
%   - Anti-windup, derivative filtering and common constraints
%   - 30 repeated Monte Carlo trials per condition
%   - 300 dpi figure export
%
% IMPORTANT: Kplant and tau are simulation assumptions, not identified
% hardware constants. Hardware validation must replace/verify them before
% physical controller certification.

clear; clc; close all;
rng(42,'twister');

%% Output directory
thisFile = mfilename('fullpath');
if isempty(thisFile)
    baseDir = pwd;
else
    baseDir = fileparts(thisFile);
end
figDir = fullfile(baseDir,'Figures_300dpi');
dataDir = fullfile(baseDir,'Data');
if ~exist(figDir,'dir'); mkdir(figDir); end
if ~exist(dataDir,'dir'); mkdir(dataDir); end

%% Core requirements and assumptions
P.Fref = 5.5;                     % N
P.forceTol = [5.0 6.0];           % N
P.dt = 0.02;                      % s, 50 Hz force loop
P.fsForce = 1/P.dt;               % Hz
P.fsGeom = 15;                    % Hz
P.T = 10.0;                       % s
P.Kmin = 780;                     % N/m, assumed envelope
P.Kmax = 900;                     % N/m, assumed envelope
P.tauMin = 0.20;                  % s, assumed envelope
P.tauMax = 0.24;                  % s, assumed envelope
P.xMin = 0.0;                     % m, local correction coordinate
P.xMax = 0.010;                   % m, 10 mm local correction range
P.xRate = 0.010;                  % m/s, 10 mm/s actuator rate limit
P.forceNoiseSigma = 0.025;        % N synthetic sensor noise
P.derivativeFc = 8.0;             % Hz derivative LPF cutoff
P.recoveryBand = 0.10;            % N
P.recoverySamples = 10;           % 0.2 s persistence at 50 Hz
P.repetitions = 30;

% Fixed controller and scheduled upper-bound gains.
% Units reflect displacement-command form:
% Kp [m/N], Ki [m/(N*s)], Kd [m*s/N].
G.fixed = [1.55e-3 1.05e-2 6.0e-5];
G.high  = [2.50e-3 1.85e-2 1.0e-4];
G.fallback = [1.20e-3 8.00e-3 4.0e-5];

rhoGrid = [0 0.25 0.50 0.75 1.00];
disturbances = {'nominal','compression','retraction','sinusoidal'};
controllers = {'fixed','scheduled'};

%% Monte Carlo validation
resultRows = [];
rowCounter = 0;
for ir = 1:numel(rhoGrid)
    rho = rhoGrid(ir);
    for id = 1:numel(disturbances)
        disturbance = disturbances{id};
        for rep = 1:P.repetitions
            trialSeed = 100000 + round(rho*1000)*1000 + rep*10 + (id-1)*10000;
            for ic = 1:numel(controllers)
                controller = controllers{ic};
                R = runForceTrial(P,G,rho,disturbance,controller,trialSeed);
                rowCounter = rowCounter + 1;
                resultRows(rowCounter,:) = [rho, id, rep, ic, R.Kplant, R.tau, ...
                    R.rmse, R.peakAbsError, R.timeInTolerance, R.recoveryTime]; %#ok<SAGROW>
            end
        end
    end
end

Results = array2table(resultRows,'VariableNames', ...
    {'rho_g','disturbance_id','repetition','controller_id','Kplant_Npm', ...
     'tau_s','force_RMSE_N','peak_abs_error_N','time_in_tolerance_pct','recovery_time_s'});
Results.disturbance = categorical(disturbances(Results.disturbance_id))';
Results.controller = categorical(controllers(Results.controller_id))';
Results = movevars(Results,{'disturbance','controller'},'After','rho_g');
writetable(Results,fullfile(dataDir,'force_controller_monte_carlo.csv'));

%% Summary table
Summary = table();
s = 0;
for ir = 1:numel(rhoGrid)
    rho = rhoGrid(ir);
    for id = 1:numel(disturbances)
        for ic = 1:numel(controllers)
            mask = Results.rho_g==rho & Results.disturbance_id==id & Results.controller_id==ic;
            s = s + 1;
            Summary.rho_g(s,1) = rho;
            Summary.disturbance{s,1} = disturbances{id};
            Summary.controller{s,1} = controllers{ic};
            Summary.rmse_mean_N(s,1) = mean(Results.force_RMSE_N(mask));
            Summary.rmse_sd_N(s,1) = std(Results.force_RMSE_N(mask));
            Summary.peak_mean_N(s,1) = mean(Results.peak_abs_error_N(mask));
            Summary.peak_sd_N(s,1) = std(Results.peak_abs_error_N(mask));
            Summary.time_in_tolerance_mean_pct(s,1) = mean(Results.time_in_tolerance_pct(mask));
            Summary.recovery_median_s(s,1) = median(Results.recovery_time_s(mask),'omitnan');
        end
    end
end
writetable(Summary,fullfile(dataDir,'force_controller_summary.csv'));

%% Figure 1: gain schedule
rhoFine = linspace(0,1,101)';
Kp = G.fixed(1) + rhoFine*(G.high(1)-G.fixed(1));
Ki = G.fixed(2) + rhoFine*(G.high(2)-G.fixed(2));
Kd = G.fixed(3) + rhoFine*(G.high(3)-G.fixed(3));

f = figure('Color','w','Position',[100 100 750 480]);
plot(rhoFine,Kp,'LineWidth',1.6); grid on;
xlabel('\rho_g'); ylabel('K_p [m/N]'); title('Scheduled proportional gain');
exportgraphics(f,fullfile(figDir,'Fig_gain_schedule_Kp.png'),'Resolution',300); close(f);

f = figure('Color','w','Position',[100 100 750 480]);
plot(rhoFine,Ki,'LineWidth',1.6); grid on;
xlabel('\rho_g'); ylabel('K_i [m/(N s)]'); title('Scheduled integral gain');
exportgraphics(f,fullfile(figDir,'Fig_gain_schedule_Ki.png'),'Resolution',300); close(f);

f = figure('Color','w','Position',[100 100 750 480]);
plot(rhoFine,Kd,'LineWidth',1.6); grid on;
xlabel('\rho_g'); ylabel('K_d [m s/N]'); title('Scheduled derivative gain');
exportgraphics(f,fullfile(figDir,'Fig_gain_schedule_Kd.png'),'Resolution',300); close(f);

%% Figure 2: representative compression-pulse comparison at rho_g = 1
seed = 13579;
RF = runForceTrial(P,G,1.0,'compression','fixed',seed);
RS = runForceTrial(P,G,1.0,'compression','scheduled',seed);
f = figure('Color','w','Position',[100 100 820 500]);
plot(RF.t,RF.force,'LineWidth',1.4); hold on;
plot(RS.t,RS.force,'LineWidth',1.4);
yline(P.Fref,'--','F_{ref}');
yline(P.forceTol(1),':','5.0 N'); yline(P.forceTol(2),':','6.0 N');
xlabel('Time [s]'); ylabel('Force [N]');
title('Compression disturbance: fixed vs scheduled PID, \rho_g = 1');
legend('Fixed PID','Scheduled PID','Location','best'); grid on; xlim([4.5 8]);
exportgraphics(f,fullfile(figDir,'Fig_force_compression_rho1.png'),'Resolution',300); close(f);

%% Figure 3: representative sinusoidal comparison at rho_g = 1
RF = runForceTrial(P,G,1.0,'sinusoidal','fixed',24680);
RS = runForceTrial(P,G,1.0,'sinusoidal','scheduled',24680);
f = figure('Color','w','Position',[100 100 820 500]);
plot(RF.t,RF.force,'LineWidth',1.4); hold on;
plot(RS.t,RS.force,'LineWidth',1.4);
yline(P.Fref,'--','F_{ref}');
yline(P.forceTol(1),':','5.0 N'); yline(P.forceTol(2),':','6.0 N');
xlabel('Time [s]'); ylabel('Force [N]');
title('2 Hz force-equivalent disturbance: fixed vs scheduled PID');
legend('Fixed PID','Scheduled PID','Location','best'); grid on; xlim([4.5 8]);
exportgraphics(f,fullfile(figDir,'Fig_force_sinusoidal_rho1.png'),'Resolution',300); close(f);

%% Figure 4: RMSE by geometry severity, non-nominal disturbances combined
rmseFixed = nan(size(rhoGrid)); rmseSched = nan(size(rhoGrid));
for ir = 1:numel(rhoGrid)
    rho = rhoGrid(ir);
    nonNominal = Results.disturbance_id~=1 & Results.rho_g==rho;
    mf = nonNominal & Results.controller_id==1;
    ms = nonNominal & Results.controller_id==2;
    rmseFixed(ir) = mean(Results.force_RMSE_N(mf));
    rmseSched(ir) = mean(Results.force_RMSE_N(ms));
end
f = figure('Color','w','Position',[100 100 820 500]);
plot(rhoGrid,rmseFixed,'-o','LineWidth',1.5); hold on;
plot(rhoGrid,rmseSched,'-s','LineWidth',1.5);
xlabel('Geometry severity \rho_g'); ylabel('Mean force RMSE [N]');
title('Force-control robustness across the assumed plant envelope');
legend('Fixed PID','Scheduled PID','Location','best'); grid on;
exportgraphics(f,fullfile(figDir,'Fig_RMSE_vs_rho.png'),'Resolution',300); close(f);

%% Console summary
fprintf('\nIRP controller validation complete.\n');
fprintf('Results written to: %s\n',dataDir);
fprintf('Figures written to: %s\n\n',figDir);
for ir = 1:numel(rhoGrid)
    red = 100*(rmseFixed(ir)-rmseSched(ir))/rmseFixed(ir);
    fprintf('rho_g = %.2f : fixed RMSE %.4f N, scheduled %.4f N, reduction %.1f %%\n', ...
        rhoGrid(ir),rmseFixed(ir),rmseSched(ir),red);
end

%% Local function
function R = runForceTrial(P,G,rho,disturbance,controller,trialSeed)
    rng(trialSeed,'twister');
    t = (0:P.dt:P.T)';

    K_nom = P.Kmax - (P.Kmax-P.Kmin)*rho;
    tau_nom = P.tauMin + (P.tauMax-P.tauMin)*rho;
    Kplant = min(P.Kmax,max(P.Kmin,K_nom*(1+0.02*randn)));
    tau = min(P.tauMax,max(P.tauMin,tau_nom*(1+0.025*randn)));

    if strcmp(controller,'fixed')
        gains = G.fixed;
    elseif strcmp(controller,'scheduled')
        gains = G.fixed + rho*(G.high-G.fixed);
    else
        gains = G.fallback;
    end
    Kp = gains(1); Ki = gains(2); Kd = gains(3);

    x_ff = P.Fref/K_nom;
    x_a = P.Fref/Kplant;
    F = P.Fref;
    integ = 0;
    ePrev = 0;
    dFilt = 0;
    alphaD = (2*pi*P.derivativeFc*P.dt)/(1+2*pi*P.derivativeFc*P.dt);
    disturbanceScale = 1 + 0.03*randn;

    force = zeros(size(t));
    xHist = zeros(size(t));
    uHist = zeros(size(t));
    dHist = zeros(size(t));

    for k = 1:numel(t)
        tk = t(k);
        dF = 0;
        switch disturbance
            case 'compression'
                if tk >= 5.0 && tk < 5.5
                    dF = 1.0*disturbanceScale;
                end
            case 'retraction'
                if tk >= 5.0 && tk < 5.5
                    dF = -1.0*disturbanceScale;
                end
            case 'sinusoidal'
                if tk >= 5.0 && tk < 7.0
                    dF = 0.75*disturbanceScale*sin(4*pi*(tk-5.0));
                end
        end

        Fm = F + P.forceNoiseSigma*randn;
        e = P.Fref - Fm;
        de = (e-ePrev)/P.dt;
        dFilt = dFilt + alphaD*(de-dFilt);

        uUnsat = x_ff + Kp*e + Ki*integ + Kd*dFilt;
        u = min(P.xMax,max(P.xMin,uUnsat));

        % Conditional integration anti-windup.
        if abs(u-uUnsat) < 1e-12 || (u>=P.xMax && e<0) || (u<=P.xMin && e>0)
            integ = integ + e*P.dt;
        end

        dx = min(P.xRate*P.dt,max(-P.xRate*P.dt,u-x_a));
        x_a = x_a + dx;

        % Assumed force plant. dF is a force-equivalent perturbation.
        Fdot = (Kplant*x_a + dF - F)/tau;
        F = F + P.dt*Fdot;

        force(k) = Fm;
        xHist(k) = x_a;
        uHist(k) = u;
        dHist(k) = dF;
        ePrev = e;
    end

    analysisMask = t>=4.8 & t<=8.0;
    err = force(analysisMask)-P.Fref;
    rmse = sqrt(mean(err.^2));
    peakAbsError = max(abs(err));
    inTol = force(analysisMask)>=P.forceTol(1) & force(analysisMask)<=P.forceTol(2);
    timeInTolerance = 100*mean(inTol);

    if strcmp(disturbance,'sinusoidal')
        tEnd = 7.0;
    elseif strcmp(disturbance,'nominal')
        tEnd = 5.0;
    else
        tEnd = 5.5;
    end
    recoveryTime = NaN;
    firstCandidate = find(t>=tEnd,1,'first');
    for k = firstCandidate:(numel(t)-P.recoverySamples+1)
        win = k:(k+P.recoverySamples-1);
        if all(abs(force(win)-P.Fref)<=P.recoveryBand)
            recoveryTime = t(k)-tEnd;
            break;
        end
    end

    R.t = t; R.force = force; R.x = xHist; R.u = uHist; R.d = dHist;
    R.Kplant = Kplant; R.tau = tau;
    R.rmse = rmse; R.peakAbsError = peakAbsError;
    R.timeInTolerance = timeInTolerance; R.recoveryTime = recoveryTime;
end
