clear; close all; clc;

%% Given Link Lengths (in mm)
AB = 10; CB = 30; CD = 10; DE = 20;
DF = 60; EG = 60; FG = 20; GH = 60;

%% Fixed Points
A = [0; 0];
D = [30; 0]; 

%% Angles at zero configuration
theta1 = 0; % Motor 1 angle
theta2 = 0; % Motor 2 angle

%% Directions
% DC vertical (up at zero)
DC_dir = [0;1];

% DE at -120° from DC
angle_DE = -120*pi/180;
R = [cos(angle_DE), -sin(angle_DE); sin(angle_DE), cos(angle_DE)];
DE_dir = R*DC_dir; 

%% Points B, C, E
B = A + AB*DC_dir;
C = D + CD*DC_dir;
E = D + DE*DE_dir;

%% For Motor 2 zero: DF vertical down
DF_dir = [0;-1];
F = D + DF*DF_dir;

% Points G and H along F in direction parallel to DE
% We know F-G = 20 mm and G-H = 60 mm. 
G = F + FG*DE_dir;
H = G + GH*DE_dir;

%% Plot
figure; hold on; grid on; 
title('Leg Mechanism at Zero Angles');
xlabel('X (mm)');
ylabel('Y (mm)');

% Ground line
y_ground = -100; 
plot([-50, 300], [y_ground, y_ground], 'k--');

% Plot Points
plot(A(1),A(2),'ko','MarkerFaceColor','k'); text(A(1), A(2)+2,'A');
plot(B(1),B(2),'ro','MarkerFaceColor','r'); text(B(1), B(2)+2,'B');
plot(C(1),C(2),'bo','MarkerFaceColor','b'); text(C(1), C(2)+2,'C');
plot(D(1),D(2),'ko','MarkerFaceColor','k'); text(D(1), D(2)-4,'D');
plot(E(1),E(2),'go','MarkerFaceColor','g'); text(E(1), E(2)-4,'E');
plot(F(1),F(2),'mo','MarkerFaceColor','m'); text(F(1), F(2)-4,'F');
plot(G(1),G(2),'co','MarkerFaceColor','c'); text(G(1), G(2)-4,'G');
plot(H(1),H(2),'yo','MarkerFaceColor','y'); text(H(1), H(2)-4,'H');

% Draw links
plot([A(1), B(1)], [A(2), B(2)], 'r-', 'LineWidth',2);
plot([B(1), C(1)], [B(2), C(2)], 'b-', 'LineWidth',2);
plot([C(1), D(1)], [C(2), D(2)], 'k-', 'LineWidth',2);
plot([D(1), E(1)], [D(2), E(2)], 'g-', 'LineWidth',2);
plot([D(1), F(1)], [D(2), F(2)], 'm-', 'LineWidth',2);
plot([F(1), G(1)], [F(2), G(2)], 'c-', 'LineWidth',2);
plot([G(1), H(1)], [G(2), H(2)], 'y-', 'LineWidth',2);

% If E and G need to be connected by a link:
plot([E(1), G(1)], [E(2), G(2)], 'k-', 'LineWidth',2);

axis equal;
axis tight;
hold off;
