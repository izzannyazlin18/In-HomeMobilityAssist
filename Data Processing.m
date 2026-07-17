close all;
clear all;

% Load the data 
data = readmatrix('data1.txt');  

% Assign columns
y_human = data(:,1);
v_trans = data(:,2);
delta_theta = data(:,3);
v_rot = data(:,4);

% Time vector based on 0.01s step size
time = (0:length(y_human)-1)' * 0.01;

% -------- Plot 1: Y Human vs Translational Hub Velocity --------
figure;
plot(time, y_human, 'b', 'LineWidth', 1.5); hold on;
plot(time, v_trans, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Linear Velocity (cm/s)');
title('Human Trans Velocity vs. Hub Trans Velocity');
legend('Human Trans Velocity ', 'Hub Trans Velocity');
grid on;

% -------- Plot 2: Delta Theta vs Rotational Hub Velocity --------
figure;
plot(time, delta_theta, 'g', 'LineWidth', 1.5); hold on;
plot(time, v_rot, 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Human Rot Velocity vs. Hub Rot Velocity');
legend('Human Rot Velocity ', 'Hub Rot Velocity');
axis([5 6.6 -5 5])
grid on;
