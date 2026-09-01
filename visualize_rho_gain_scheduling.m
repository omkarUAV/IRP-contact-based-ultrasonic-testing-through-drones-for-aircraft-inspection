%% Gain-scheduling visualisation: raw, filtered and clamped rho
clear; close all; clc;

%% Simulation settings
Ts = 0.02;                         % Sample time [s]
t  = (0:Ts:30).';                  % Time vector [s]

rhoMin = 0;                        % Minimum validated rho [deg]
rhoMax = 30;                       % Maximum validated rho [deg]

%% Example raw scheduling-variable signal
% Replace rhoRaw with your measured/Simulink-logged scheduling variable.
rng(7);
rhoRaw = 12 ...
       + 13*sin(2*pi*0.055*t) ...
       +  7*(t >= 8 & t < 14) ...
       - 10*(t >= 20 & t < 24) ...
       +  1.3*randn(size(t));

%% First-order low-pass filter (no additional toolbox required)
filterTimeConstant = 0.35;         % Filter time constant [s]
alpha = Ts/(filterTimeConstant + Ts);

rhoFiltered = zeros(size(rhoRaw));
rhoFiltered(1) = rhoRaw(1);

for k = 2:numel(t)
    rhoFiltered(k) = rhoFiltered(k-1) ...
                   + alpha*(rhoRaw(k) - rhoFiltered(k-1));
end

%% Experimental-envelope protection
outOfRange = rhoFiltered < rhoMin | rhoFiltered > rhoMax;
rhoClamped = min(max(rhoFiltered, rhoMin), rhoMax);

%% Experimentally identified gain table
rhoBreakpoints = [0 10 20 30];     % Validated operating points [deg]
KpTable = [1.10 1.35 1.75 2.15];
KiTable = [0.48 0.56 0.70 0.86];
KdTable = [0.08 0.10 0.13 0.17];

% Linear interpolation is safe here because rhoClamped cannot leave the
% experimentally validated breakpoint range.
Kp = interp1(rhoBreakpoints, KpTable, rhoClamped, 'linear');
Ki = interp1(rhoBreakpoints, KiTable, rhoClamped, 'linear');
Kd = interp1(rhoBreakpoints, KdTable, rhoClamped, 'linear');

%% Visual dashboard
figure('Color','w', ...
       'Name','Geometry-Guided PID Gain Scheduling', ...
       'Position',[100 60 1200 900]);

layout = tiledlayout(5,1,'TileSpacing','compact','Padding','compact');
title(layout,'Scheduling Variable and Scheduled PID Gains', ...
      'FontWeight','bold');

nexttile;
plot(t,rhoRaw,'Color',[0.70 0.70 0.70],'LineWidth',0.9); hold on;
plot(t,rhoFiltered,'Color',[0.00 0.45 0.74],'LineWidth',1.5);
plot(t,rhoClamped,'Color',[0.85 0.33 0.10],'LineWidth',1.8);
yline(rhoMin,'k--','Minimum validated');
yline(rhoMax,'k--','Maximum validated');
grid on; ylabel('\rho [deg]');
legend('Raw \rho','Filtered \rho','Clamped \rho', ...
       'Location','eastoutside');

nexttile;
stairs(t,double(outOfRange),'Color',[0.64 0.08 0.18],'LineWidth',1.5);
ylim([-0.1 1.1]); yticks([0 1]);
yticklabels({'Inside','Outside'});
grid on; ylabel('Range flag');

nexttile;
plot(t,Kp,'Color',[0.47 0.67 0.19],'LineWidth',1.7);
grid on; ylabel('K_p');

nexttile;
plot(t,Ki,'Color',[0.49 0.18 0.56],'LineWidth',1.7);
grid on; ylabel('K_i');

nexttile;
plot(t,Kd,'Color',[0.93 0.69 0.13],'LineWidth',1.7);
grid on; ylabel('K_d'); xlabel('Time [s]');

linkaxes(findall(gcf,'Type','axes'),'x');
xlim([t(1) t(end)]);

