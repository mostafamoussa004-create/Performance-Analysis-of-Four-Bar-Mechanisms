clear all;
clc;

% Four-Bar Mechanism Simulation and Analysis
% Based on Example 1: r1=100, r2=30, r3=85.4, r4=60
clear; clc; close all;

%% 1. Define Mechanism Parameters (Link Lengths)
r1 = 100;   % Fixed link (O2 to O4)
r2 = 30;    % Crank (Driver)
r3 = 85.4;  % Coupler
r4 = 60;    % Follower (Rocker)

% Crank rotation range (Full revolution)
theta2 = linspace(0, 2*pi, 100); 

% Pre-allocate arrays
theta3 = zeros(size(theta2));
theta4 = zeros(size(theta2));
gamma = zeros(size(theta2)); % Transmission Angle

%% 2. Command Window Calculations
fprintf('--- Mechanism Parameters & Analysis ---\n');

% Calculate Degrees of Freedom (DOF) using Grubler's Criterion
% For a 4-bar: n=4 links, j=4 revolute joints
n = 4; j = 4;
dof = 3*(n-1) - 2*j;
fprintf('Degrees of Freedom (DOF): %d\n', dof);

% Grashof Law Check
links = sort([r1, r2, r3, r4]);
S = links(1); L = links(4); P = links(2); Q = links(3);
if (S + L) <= (P + Q)
    fprintf('Grashof Condition: S+L <= P+Q (Grashof - Linkages can rotate)\n');
else
    fprintf('Grashof Condition: S+L > P+Q (Non-Grashof)\n');
end

% Ratios
fprintf('Crank to Rocker Length Ratio: %.2f\n', r2/r4);
fprintf('Link Lengths: r1=%.1f, r2=%.1f, r3=%.1f, r4=%.1f\n\n', r1, r2, r3, r4);

%% 3. Position and Transmission Angle Analysis
for i = 1:length(theta2)
    % Freudenstein's Constants
    K1 = r1/r2; K2 = r1/r4;
    K3 = (r1^2 + r2^2 - r3^2 + r4^2) / (2*r2*r4);
    
    A = cos(theta2(i)) - K1 - K2*cos(theta2(i)) + K3;
    B = -2*sin(theta2(i));
    C = K1 - (K2+1)*cos(theta2(i)) + K3;
    
    % Follower Angle (theta4)
    theta4(i) = 2 * atan((-B - sqrt(B^2 - 4*A*C)) / (2*A));
    
    % Transmission Angle (gamma)
    BD = sqrt(r1^2 + r2^2 - 2*r1*r2*cos(theta2(i)));
    gamma(i) = acos((r3^2 + r4^2 - BD^2) / (2*r3*r4));
end

% Display Performance Values
fprintf('--- Angular Performance ---\n');
fprintf('Follower Range (Rocker Swing): %.2f deg to %.2f deg\n', ...
    min(rad2deg(theta4)), max(rad2deg(theta4)));
fprintf('Min Transmission Angle: %.2f deg\n', min(rad2deg(gamma)));
fprintf('Max Transmission Angle: %.2f deg\n', max(rad2deg(gamma)));

%% 4. Performance Visualization (Animation)
figure('Name', 'Four-Bar Mechanism Analysis');
subplot(2,1,1);
hold on; grid on; axis equal;
title('Mechanism Animation');
xlim([-50, 150]); ylim([-80, 80]);

% Static Pinned Joints
plot(0, 0, 'ko', 'MarkerFaceColor', 'k'); 
plot(r1, 0, 'ko', 'MarkerFaceColor', 'k'); 

h_links = plot([0 0 0 0], [0 0 0 0], '-o', 'LineWidth', 2);

for i = 1:length(theta2)
    Bx = r2 * cos(theta2(i)); By = r2 * sin(theta2(i));
    Cx = r1 + r4 * cos(theta4(i)); Cy = r4 * sin(theta4(i));
    
    set(h_links, 'XData', [0 Bx Cx r1], 'YData', [0 By Cy 0]);
    drawnow;
    pause(0.02);
end

%% 5. Transmission Angle Plot
subplot(2,1,2);
plot(rad2deg(theta2), rad2deg(gamma), 'm', 'LineWidth', 1.5);
title('Transmission Angle Analysis');
xlabel('Crank Angle (degrees)');
ylabel('Gamma (degrees)');
grid on;