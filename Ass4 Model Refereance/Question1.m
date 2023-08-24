clear;
clc;
close all;
syms s t;

% Plant transfer function G(s)
G = 1 / (s + 1); % 1 / (s + 1)

% Time interval and step size for simulation [0,30] with step 0.01
t_start = 0;
t_end = 30;
dt = 0.1;
time = t_start:dt:t_end;

% Control input uc(t) - sin wave
uc = sin(t);

% Compute the Laplace transform of the control input uc(t)
UC = laplace(uc, t, s);

% Compute the desired output Ym(s) = UC(s) * ko * G(s)
ko = 2;
YM = UC * ko * G;

% Perform the inverse Laplace transform for Ym(s) to get ym(t)
ym = ilaplace(YM, s, t);

% Evaluate ym(t) numerically for the given time range
ym_numerical = double(subs(ym, t, time));

% Perform the inverse Laplace transform for G(s) to get g(t)
g = ilaplace(G, s, t);

% Calculate and plot the system response y(t) and theta1 with time
gamma1 = 0.5;
k = 1;
theta1 = zeros(size(time));
theta1(1) = 0; % Initial condition for theta1

y1 = zeros(size(time));
y1(1) = 0; % Initial condition for y

error1 = zeros(size(time));
error1(1) = 0; % Initial condition for y

for i = 2:length(time)
    error1(i-1) = (y1(i-1) - ym_numerical(i-1));
    theta1(i) = theta1(i-1) + ym_numerical(i) * error1(i-1) * (-gamma1);
    Y1 = UC * k * G * (theta1(i));
    y_inv1 = ilaplace(Y1,s,t);
    y1(i)=subs(y_inv1,t,time(i));
    %y(i) = subs(uc, t, time(i)) * theta1(i) * k * subs(g, t, time(i));
end

%gamma is 1
gamma2 = 1;

theta2 = zeros(size(time));
theta2(1) = 0; % Initial condition for theta1

y2 = zeros(size(time));
y2(1) = 0; % Initial condition for y

error2 = zeros(size(time));
error2(1) = 0; % Initial condition for y

for i = 2:length(time)
    error2(i-1) = (y2(i-1) - ym_numerical(i-1));
    theta2(i) = theta2(i-1) + ym_numerical(i) * error2(i-1) * (-gamma2);
    Y2 = UC * k * G * (theta2(i));
    y_inv2 = ilaplace(Y2,s,t);
    y2(i)=subs(y_inv2,t,time(i));
    %y(i) = subs(uc, t, time(i)) * theta1(i) * k * subs(g, t, time(i));
end

%gamma is 2
gamma3 = 2;

theta3 = zeros(size(time));
theta3(1) = 0; % Initial condition for theta1

y3 = zeros(size(time));
y3(1) = 0; % Initial condition for y

error3 = zeros(size(time));
error3(1) = 0; % Initial condition for y

for i = 2:length(time)
    error3(i-1) = (y3(i-1) - ym_numerical(i-1));
    theta3(i) = theta3(i-1) + ym_numerical(i) * error3(i-1) * (-gamma3);
    Y3 = UC * k * G * (theta3(i));
    y_inv3 = ilaplace(Y3,s,t);
    y3(i)=subs(y_inv3,t,time(i));
    %y(i) = subs(uc, t, time(i)) * theta1(i) * k * subs(g, t, time(i));
end

% Zoom plot
% Plot theta with time
figure;
line([time(1), time(end)], [2, 2], 'Color', 'red', 'LineWidth', 1.5 , 'DisplayName', 'Desired Theta');
hold on;
plot(time, theta1, 'm--', 'LineWidth', 1.5, 'DisplayName', 'gamma=0.5 Theta');
plot(time, theta2, 'b:', 'LineWidth', 1.5, 'DisplayName', 'gamma=1 Theta');
plot(time, theta3, 'g-.', 'LineWidth', 1.5, 'DisplayName', 'gamma=2 Theta');
hold off;
xlabel('Time (s)');
ylabel('theta');
title('theta with Time');
grid on;
legend('Location', 'best');

% Set x-axis limits to zoom in on the first 3.5 seconds
xlim([0, 3.5]);

% Plot ym, y with time
figure;
plot(time, ym_numerical, 'r', 'LineWidth', 1.5, 'DisplayName', 'Desired Output y_m(t)');
hold on;
plot(time, y1, 'm--', 'LineWidth', 1.5, 'DisplayName', 'gamma=0.5 y(t)');
plot(time, y2, 'b:', 'LineWidth', 1.5, 'DisplayName', 'gamma=1 y(t)');
plot(time, y3, 'g-.', 'LineWidth', 1.5, 'DisplayName', 'gamma=2  y(t)');
%plot(time, y + ym_numerical, 'm--', 'LineWidth', 1.5, 'DisplayName', 'y(t) + y_m(t)');
hold off;
xlabel('Time (s)');
ylabel('Response');
title('System Response and Desired Output');
grid on;
legend('Location', 'best');

% Set x-axis limits to zoom in on the first 3.5 seconds
xlim([0, 3.5]);


% Plot theta with time
figure;
line([time(1), time(end)], [2, 2], 'Color', 'red', 'LineWidth', 1.5 , 'DisplayName', 'Desired Theta');
hold on;
plot(time, theta1, 'm--', 'LineWidth', 1.5, 'DisplayName', 'gamma=0.5 Theta');

plot(time, theta2, 'b:', 'LineWidth', 1.5, 'DisplayName', 'gamma=1 Theta');
plot(time, theta3, 'g-.', 'LineWidth', 1.5, 'DisplayName', 'gamma=2 Theta');
hold off;
xlabel('Time (s)');
ylabel('theta');
title('theta with Time');
grid on;
legend('Location', 'best');

% Plot ym, y with time
figure;
plot(time, ym_numerical, 'r', 'LineWidth', 1.5, 'DisplayName', 'Desired Output y_m(t)');
hold on;
plot(time, y1, 'm--', 'LineWidth', 1.5, 'DisplayName', 'gamma=0.5 y(t)');
plot(time, y2, 'b:', 'LineWidth', 1.5, 'DisplayName', 'gamma=1 y(t)');
plot(time, y3, 'g-.', 'LineWidth', 1.5, 'DisplayName', 'gamma=2  y(t)');
%plot(time, y + ym_numerical, 'm--', 'LineWidth', 1.5, 'DisplayName', 'y(t) + y_m(t)');
hold off;
xlabel('Time (s)');
ylabel('Response');
title('System Response and Desired Output');
grid on;
legend('Location', 'best');


