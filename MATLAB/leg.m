clear; clc; close all;

%% Given Link Lengths (in mm)
AB = 10; CB = 30; CD = 10; DE = 20;
DF = 60; EG = 60; FG = 20; GH = 60;

%% Fixed Points
A = [0; 0];
D = [30; 0]; % Adjust as needed

%% Ellipse Parameters for Foot H
x0 = 60;   % center x
y0 = -60;  % center y
a = 20;    % semi-major axis (x-direction)
b = 10;    % semi-minor axis (y-direction)
omega = 2*pi/5; % period of 5 seconds
t_final = 5;
dt = 0.05;
t_vals = 0:dt:t_final;

%% Storage for plotting after simulation
theta1_vals = zeros(size(t_vals));
theta2_vals = zeros(size(t_vals));
Hx_vals = zeros(size(t_vals));
Hy_vals = zeros(size(t_vals));
kneeAngles = zeros(size(t_vals));
hipAngles = zeros(size(t_vals));

%% Set up figure for animation
fig = figure('Name','Leg Ellipse Animation','NumberTitle','off');
ax = axes('Parent', fig); hold(ax, 'on');
grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'X (mm)');
ylabel(ax, 'Y (mm)');
title(ax, 'Leg Mechanism Tracing an Ellipse');
plot(ax, [-50, 300], [-100, -100], 'k--'); % ground line
xlim(ax, [-20, 200]);
ylim(ax, [-120, 50]);

% Plot placeholders for points
hA = plot(ax, 0,0,'ko','MarkerFaceColor','k');
hB = plot(ax, 0,0,'ro','MarkerFaceColor','r');
hC = plot(ax, 0,0,'bo','MarkerFaceColor','b');
hD = plot(ax, 0,0,'ko','MarkerFaceColor','k');
hE = plot(ax, 0,0,'go','MarkerFaceColor','g');
hF = plot(ax, 0,0,'mo','MarkerFaceColor','m');
hG = plot(ax, 0,0,'co','MarkerFaceColor','c');
hH = plot(ax, 0,0,'yo','MarkerFaceColor','y');

% Line handles
hLineAB = plot(ax, [0,0],[0,0],'r-','LineWidth',2);
hLineBC = plot(ax, [0,0],[0,0],'b-','LineWidth',2);
hLineCD = plot(ax, [0,0],[0,0],'k-','LineWidth',2);
hLineDE = plot(ax, [0,0],[0,0],'g-','LineWidth',2);
hLineDF = plot(ax, [0,0],[0,0],'m-','LineWidth',2);
hLineFG = plot(ax, [0,0],[0,0],'c-','LineWidth',2);
hLineGH = plot(ax, [0,0],[0,0],'y-','LineWidth',2);
hLineEG = plot(ax, [0,0],[0,0],'k-','LineWidth',2);

legend(ax,{'A','B','C','D','E','F','G','H'},'Location','bestoutside');

%% Animation Loop
for i = 1:length(t_vals)
    t = t_vals(i);
    % Desired H position on ellipse
    xH_des = x0 + a*cos(omega*t);
    yH_des = y0 + b*sin(omega*t);

    % Inverse Kinematics: find theta1 and theta2 for (xH_des, yH_des)
    [theta1, theta2] = inverseKinematicsDummy(xH_des, yH_des);

    % Forward Kinematics
    [Apos, Bpos, Cpos, Dpos, Epos, Fpos, Gpos, Hpos] = forwardKinematics(...
        A, D, AB, CB, CD, DE, DF, EG, FG, GH, theta1, theta2);

    % Update plot data
    set(hA, 'XData', Apos(1), 'YData', Apos(2));
    set(hB, 'XData', Bpos(1), 'YData', Bpos(2));
    set(hC, 'XData', Cpos(1), 'YData', Cpos(2));
    set(hD, 'XData', Dpos(1), 'YData', Dpos(2));
    set(hE, 'XData', Epos(1), 'YData', Epos(2));
    set(hF, 'XData', Fpos(1), 'YData', Fpos(2));
    set(hG, 'XData', Gpos(1), 'YData', Gpos(2));
    set(hH, 'XData', Hpos(1), 'YData', Hpos(2));

    % Update lines
    set(hLineAB, 'XData', [Apos(1) Bpos(1)], 'YData', [Apos(2) Bpos(2)]);
    set(hLineBC, 'XData', [Bpos(1) Cpos(1)], 'YData', [Bpos(2) Cpos(2)]);
    set(hLineCD, 'XData', [Cpos(1) Dpos(1)], 'YData', [Cpos(2) Dpos(2)]);
    set(hLineDE, 'XData', [Dpos(1) Epos(1)], 'YData', [Dpos(2) Epos(2)]);
    set(hLineDF, 'XData', [Dpos(1) Fpos(1)], 'YData', [Dpos(2) Fpos(2)]);
    set(hLineFG, 'XData', [Fpos(1) Gpos(1)], 'YData', [Fpos(2) Gpos(2)]);
    set(hLineGH, 'XData', [Gpos(1) Hpos(1)], 'YData', [Gpos(2) Hpos(2)]);
    set(hLineEG, 'XData', [Epos(1) Gpos(1)], 'YData', [Epos(2) Gpos(2)]);

    drawnow;

    % Store values for later plotting
    theta1_vals(i) = theta1;
    theta2_vals(i) = theta2;
    Hx_vals(i) = Hpos(1);
    Hy_vals(i) = Hpos(2);

    % Compute knee angle (angle between FD and FG at point F)
    FD_vec = Dpos - Fpos;
    FG_vec = Gpos - Fpos;
    knee_angle_rad = angleBetweenVectors(FD_vec, FG_vec);
    kneeAngles(i) = knee_angle_rad*180/pi;

    % Compute hip angle (angle of DF relative to horizontal)
    DF_vec = Fpos - Dpos;
    hip_angle_rad = atan2(DF_vec(2), DF_vec(1));
    hipAngles(i) = hip_angle_rad*180/pi;
end

%% Additional Plots for Debugging
figure('Name','Debugging Plots','NumberTitle','off');

% 1: XY plot of the foot position (H)
subplot(3,2,1);
plot(Hx_vals, Hy_vals,'b-','LineWidth',2);
grid on; axis equal;
xlabel('X (mm)');
ylabel('Y (mm)');
title('Foot Trajectory (XY)');

% 2: X position over time of the foot
subplot(3,2,2);
plot(t_vals, Hx_vals, 'r-','LineWidth',2);
grid on;
xlabel('Time (s)');
ylabel('X_H (mm)');
title('Foot X Position Over Time');

% 3: Y position over time of the foot
subplot(3,2,3);
plot(t_vals, Hy_vals, 'g-','LineWidth',2);
grid on;
xlabel('Time (s)');
ylabel('Y_H (mm)');
title('Foot Y Position Over Time');

% 4: Knee angle over time in degrees (angle between FD and FG)
subplot(3,2,4);
plot(t_vals, kneeAngles, 'm-','LineWidth',2);
grid on;
xlabel('Time (s)');
ylabel('Knee Angle (deg)');
title('Knee Angle Over Time');

% 5: Hip angle over time in deg (Angle of DF)
subplot(3,2,5);
plot(t_vals, hipAngles, 'c-','LineWidth',2);
grid on;
xlabel('Time (s)');
ylabel('Hip Angle (deg)');
title('Hip Angle Over Time');

% 6: Motor angles over time (for reference)
subplot(3,2,6);
plot(t_vals, theta1_vals*180/pi, 'r-','LineWidth',2); hold on;
plot(t_vals, theta2_vals*180/pi, 'b-','LineWidth',2);
grid on;
xlabel('Time (s)');
ylabel('Motor Angles (deg)');
legend('Motor 1','Motor 2');
title('Motor Angles Over Time');


%% Dummy Inverse Kinematics Function (Replace with real IK)
function [theta1, theta2] = inverseKinematicsDummy(xH, yH)
    % Placeholder: NOT actual IK
    theta1 = 0.2*sin(0.1*xH) + 0.1*cos(0.1*yH); 
    theta2 = 0.1*sin(0.05*xH) - 0.1*sin(0.05*yH);
end

%% Forward Kinematics Function (Adjust as needed for real geometry)
function [Apos, Bpos, Cpos, Dpos, Epos, Fpos, Gpos, Hpos] = forwardKinematics(...
    A, D, AB, CB, CD, DE, DF, EG, FG, GH, theta1, theta2)

    Apos = A;
    Dpos = D;
    % DC_dir = vertical rotated by theta1
    DC_dir = [sin(theta1); cos(theta1)];
    Bpos = Apos + AB*DC_dir;
    Cpos = Dpos + CD*DC_dir;

    angle_DE = -120*pi/180;
    R = [cos(angle_DE) -sin(angle_DE); sin(angle_DE) cos(angle_DE)];
    DE_dir = R*DC_dir;
    Epos = Dpos + DE*DE_dir;

    DF_dir0 = [0; -1];
    R2 = [cos(theta2), -sin(theta2); sin(theta2), cos(theta2)];
    DF_dir = R2*DF_dir0;
    Fpos = Dpos + DF*DF_dir;

    % Assume F-G-H parallel to DE
    Gpos = Fpos + FG*DE_dir;
    Hpos = Gpos + GH*DE_dir;
end

%% Helper function to compute angle between two vectors
function angle_rad = angleBetweenVectors(v1, v2)
    % Angle between v1 and v2
    % angle = arccos((v1·v2)/(|v1||v2|))
    dotp = dot(v1,v2);
    mag1 = norm(v1);
    mag2 = norm(v2);
    angle_rad = acos(dotp/(mag1*mag2));
end
