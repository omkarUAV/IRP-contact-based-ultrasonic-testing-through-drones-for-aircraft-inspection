%% UAV-UT Curved Surface Visual Simulation
% Contact force constrained to 3-6 N and probe angle swept from -30 to +30 deg
% Output: visual animation + contact force + probe angle + UT quality dashboard

clear; clc; close all;

%% Simulation parameters
T = 12;
dt = 0.03;
t = 0:dt:T;
N = numel(t);

R = 3.0;
scan_length = 4.5;

probe_length = 0.75;
standoff = 0.58;

F_min = 3.0;              % lower acceptable contact force [N]
F_max = 6.0;              % upper acceptable contact force [N]
F_target = 4.5;           % centre target force [N]

theta_min = -30;          % lower angle limit [deg]
theta_max = 30;           % upper angle limit [deg]

%% Flipped curved aircraft surface: convex upper fuselage
x = linspace(-scan_length/2, scan_length/2, N);
surface_y = sqrt(R^2 - x.^2) - 2.35;

%% UAV path above surface
uav_y = surface_y + standoff + 0.035*sin(2*pi*1.2*t);

%% Contact force model: constrained inside 3-6 N
F_contact_raw = F_target + 1.25*sin(2*pi*0.55*t) + 0.35*sin(2*pi*1.7*t);
F_contact = min(max(F_contact_raw, F_min), F_max);

%% Probe angle model: swept between -30 and +30 deg
theta = 30*sin(2*pi*0.18*t);

%% UT quality model
% Best quality occurs near F_target and 0 deg angle.
% Quality degrades as force moves away from 4.5 N and angle approaches +/-30 deg.
force_error_norm = abs(F_contact - F_target) ./ ((F_max - F_min)/2);
angle_error_norm = abs(theta) ./ max(abs([theta_min theta_max]));

Q = exp(-0.85*force_error_norm.^2) .* exp(-1.75*angle_error_norm.^2);
Q = max(min(Q,1),0);

%% Numerical summary
fprintf('\n===== UAV-UT QUALITY CHECK =====\n');
fprintf('Contact force range used: %.2f to %.2f N\n', min(F_contact), max(F_contact));
fprintf('Probe angle range used: %.2f to %.2f deg\n', min(theta), max(theta));
fprintf('Mean UT quality: %.3f\n', mean(Q));
fprintf('Minimum UT quality: %.3f\n', min(Q));
fprintf('Maximum UT quality: %.3f\n', max(Q));
fprintf('Good quality percentage Q >= 0.75: %.1f %%\n', 100*mean(Q >= 0.75));
fprintf('Medium quality percentage 0.45 <= Q < 0.75: %.1f %%\n', 100*mean(Q >= 0.45 & Q < 0.75));
fprintf('Poor quality percentage Q < 0.45: %.1f %%\n\n', 100*mean(Q < 0.45));

%% Figure setup
fig = figure('Color','w','Position',[80 80 1320 760]);
set(fig,'Name','UAV-Based Ultrasonic Probe Contact Simulation');

for i = 1:N

    clf;

    %% Main scene
    subplot(2,3,[1 2 4 5]);
    hold on; grid on; axis equal;

    xs = linspace(-2.6,2.6,350);
    ys = sqrt(R^2 - xs.^2) - 2.35;

    % Aircraft fuselage skin
    plot(xs,ys,'Color',[0.05 0.05 0.05],'LineWidth',4);
    fill([xs fliplr(xs)], [ys -0.9*ones(size(ys))], ...
        [0.90 0.93 0.96], 'FaceAlpha',0.65, 'EdgeColor','none');

    drone_x = x(i);
    drone_y = uav_y(i);
    contact_x = drone_x;
    contact_y = surface_y(i);

    % UT quality colour
    if Q(i) >= 0.75
        qcol = [0.00 0.60 0.20];
        qtxt = 'GOOD';
    elseif Q(i) >= 0.45
        qcol = [0.95 0.55 0.05];
        qtxt = 'MEDIUM';
    else
        qcol = [0.85 0.05 0.05];
        qtxt = 'POOR';
    end

    % Inspection trail
    plot(x(1:i), surface_y(1:i), 'Color',[0.1 0.45 0.9], 'LineWidth',2);

    % Probe line
    plot([drone_x contact_x], [drone_y contact_y], ...
        'Color',[0.85 0.10 0.10], 'LineWidth',4);

    % Spring rings on probe
    nSpring = 6;
    sy = linspace(contact_y, drone_y, nSpring);
    for s = 2:nSpring-1
        plot(drone_x + 0.035*(-1)^s, sy(s), 'ko', ...
            'MarkerFaceColor',[0.95 0.95 0.95], 'MarkerSize',4);
    end

    % Contact point
    scatter(contact_x,contact_y,120,'filled', ...
        'MarkerFaceColor',[1.0 0.85 0.05], ...
        'MarkerEdgeColor','k','LineWidth',1.2);

    % Force vector
    quiver(contact_x, contact_y, 0, -0.22*F_contact(i)/F_target, ...
        0, 'Color',[0.85 0 0], 'LineWidth',2.2, 'MaxHeadSize',2);

    % UAV body
    rectangle('Position',[drone_x-0.25, drone_y-0.08, 0.50, 0.16], ...
        'Curvature',0.25, ...
        'FaceColor',[0.72 0.75 0.78], ...
        'EdgeColor',[0.1 0.1 0.1], ...
        'LineWidth',1.4);

    % UAV arms
    plot([drone_x-0.55 drone_x+0.55],[drone_y drone_y], ...
        'Color',[0.05 0.05 0.05],'LineWidth',2.2);
    plot([drone_x drone_x],[drone_y-0.28 drone_y+0.28], ...
        'Color',[0.05 0.05 0.05],'LineWidth',2.2);

    % Rotors
    drawRotor(drone_x-0.55, drone_y, 0.10);
    drawRotor(drone_x+0.55, drone_y, 0.10);
    drawRotor(drone_x, drone_y-0.28, 0.10);
    drawRotor(drone_x, drone_y+0.28, 0.10);

    % UT status beacon
    scatter(drone_x, drone_y+0.45, 240, 'filled', ...
        'MarkerFaceColor', qcol, 'MarkerEdgeColor','k', 'LineWidth',1.1);
    text(drone_x+0.18, drone_y+0.45, ['UT: ' qtxt], ...
        'FontSize',13,'FontWeight','bold','Color',qcol);

    % Dashboard text
    info = sprintf(['Time: %.2f s\n' ...
                    'Contact Force: %.2f N\n' ...
                    'Allowed Force: %.1f-%.1f N\n' ...
                    'Probe Angle: %.2f deg\n' ...
                    'Allowed Angle: %.0f to %.0f deg\n' ...
                    'UT Quality: %.2f'], ...
                    t(i), F_contact(i), F_min, F_max, theta(i), theta_min, theta_max, Q(i));

    text(-2.85, 1.18, info, ...
        'FontSize',11, ...
        'BackgroundColor',[1 1 1 0.90], ...
        'EdgeColor',[0.75 0.75 0.75], ...
        'Margin',8);

    xlabel('Aircraft Surface X [m]');
    ylabel('Height [m]');
    title('UAV-Based Ultrasonic Probe Contact on Curved Aircraft Surface', ...
        'FontSize',14,'FontWeight','bold');

    xlim([-3.1 3.1]);
    ylim([-0.9 1.72]);

    %% Force plot
    subplot(3,3,3);
    plot(t(1:i),F_contact(1:i),'LineWidth',1.8,'Color',[0.1 0.25 0.75]);
    hold on;
    yline(F_min,'--','3 N lower','LineWidth',1.1);
    yline(F_target,'-','4.5 N target','LineWidth',1.2);
    yline(F_max,'--','6 N upper','LineWidth',1.1);
    grid on;
    xlabel('Time [s]');
    ylabel('Force [N]');
    title('Contact Force: 3-6 N');
    xlim([0 T]);
    ylim([2.5 6.5]);

    %% Angle plot
    subplot(3,3,6);
    plot(t(1:i),theta(1:i),'LineWidth',1.8,'Color',[0.55 0.15 0.75]);
    hold on;
    yline(theta_min,'--','-30 deg','LineWidth',1.1);
    yline(0,'-','0 deg','LineWidth',1.0);
    yline(theta_max,'--','+30 deg','LineWidth',1.1);
    grid on;
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    title('Probe Angle: -30 to +30 deg');
    xlim([0 T]);
    ylim([-35 35]);

    %% UT quality plot
    subplot(3,3,9);
    plot(t(1:i),Q(1:i),'LineWidth',1.8,'Color',[0.0 0.55 0.22]);
    hold on;
    yline(0.75,'--','Good limit');
    yline(0.45,'--','Poor limit');
    grid on;
    xlabel('Time [s]');
    ylabel('Quality Index');
    title('UT Signal Quality');
    xlim([0 T]);
    ylim([0 1.05]);

    drawnow;
end

%% Final force-angle quality map
figure('Color','w','Position',[150 120 900 620]);
force_grid = linspace(F_min,F_max,120);
angle_grid = linspace(theta_min,theta_max,120);
[FG,AG] = meshgrid(force_grid,angle_grid);

FE_norm = abs(FG - F_target) ./ ((F_max - F_min)/2);
AE_norm = abs(AG) ./ max(abs([theta_min theta_max]));
Qmap = exp(-0.85*FE_norm.^2).*exp(-1.75*AE_norm.^2);

contourf(FG,AG,Qmap,24,'LineColor','none');
colorbar;
hold on;
plot(F_contact,theta,'w','LineWidth',1.6);
xlabel('Contact Force [N]');
ylabel('Probe Angle [deg]');
title('UT Quality Map for Contact Force 3-6 N and Angle -30 to +30 deg');
grid on;

%% Local function
function drawRotor(cx,cy,r)
    ang = linspace(0,2*pi,60);
    plot(cx + r*cos(ang), cy + r*sin(ang), ...
        'Color',[0.1 0.35 0.95],'LineWidth',1.4);
    fill(cx + r*cos(ang), cy + r*sin(ang), ...
        [0.75 0.85 1.0], 'FaceAlpha',0.45, 'EdgeColor','none');
end
