clear;
clc;
close all;
syms s t;

% Time interval and step size for simulation [0,100] with step 0.1
t_start = 0;
t_end = 100;
dt = 0.5;
time = t_start:dt:t_end;

% Reference model input uc(t) - sin wave
%uc = square(2*pi*(1/20)*t);  % Updated to match the size of theta1_values and theta2_values
uc = sin(t);
% Compute the Laplace transform of the reference model input uc(t)
UC = laplace(uc, t, s);

uc = ilaplace(UC, s, t);

% Compute the desired output Ym(s) = 2 * (UC(s) - ym(s))
YM = (2/(s+2)) * (UC);

% Perform the inverse Laplace transform for Ym(s) to get ym(t)
ym = ilaplace(YM, s, t);

% Evaluate ym(t) numerically for the given time range
ym_numerical = double(subs(ym, t, time));

% Initialize arrays to store controller values and system outputs
gamma1 = 0.5;

theta11 = zeros(size(time));
theta21 = zeros(size(time));
theta11(1) = 0;
theta21(1) = 0;

dtheta11 = zeros(size(time));
dtheta21 = zeros(size(time));
dtheta11(1) = 0;
dtheta21(1) = 0;

y1 = zeros(size(time));
y1(1) = 0; % Initial condition for y

% Initial conditions for theta1 and theta2


error1 = zeros(size(time));
error1(1) = 0;

% Simulate the system over time
for i = 2:length(time)
    % Calculate the current error
    error1(i) = y1(i-1) - ym_numerical(i-1);
    
    dtheta11 = (0.5/(s-0.5*theta21(i-1)+1)) * UC ;
    dtheta21 = ((0.5*0.5*theta11(i-1))/((s-0.5*theta21(i-1)+1)^2)) * UC ;

    dtheta11 = ilaplace(dtheta11, s, t);
    dtheta11 = double(subs(dtheta11, t, time(i)));
    
    dtheta21 = ilaplace(dtheta21, s, t);
    dtheta21 = double(subs(dtheta21, t, time(i)));

    % Update theta1 and theta2 using the MIT rule
    theta11(i) = theta11(i-1) + error1(i) * (-gamma1) * dtheta11;
    theta21(i) = theta21(i-1) + (-gamma1) * error1(i) * dtheta21;
    
    % Calculate the control input u(t) and system output y(t) at the next time step
    Y1 =( (0.5*theta11(i))/(s-0.5*theta21(i)+1) )* UC;
    y_inv1 = ilaplace(Y1, s, t);
    y1(i) = double(subs(y_inv1, t, time(i)));
end


% Initialize arrays to store controller values and system outputs
gamma2 = 1;

theta12 = zeros(size(time));
theta22 = zeros(size(time));
theta12(1) = 0;
theta22(1) = 0;

dtheta12 = zeros(size(time));
dtheta22 = zeros(size(time));
dtheta12(1) = 0;
dtheta22(1) = 0;

y2 = zeros(size(time));
y2(1) = 0; % Initial condition for y

% Initial conditions for theta1 and theta2


error2 = zeros(size(time));
error2(1) = 0;

% Simulate the system over time
for i = 2:length(time)
    % Calculate the current error
    error2(i) = y2(i-1) - ym_numerical(i-1);
    
    dtheta12 = (0.5/(s-0.5*theta22(i-1)+1)) * UC ;
    dtheta22 = ((0.5*0.5*theta12(i-1))/((s-0.5*theta22(i-1)+1)^2)) * UC ;

    dtheta12 = ilaplace(dtheta12, s, t);
    dtheta12 = double(subs(dtheta12, t, time(i)));
    
    dtheta22 = ilaplace(dtheta22, s, t);
    dtheta22 = double(subs(dtheta22, t, time(i)));

    % Update theta1 and theta2 using the MIT rule
    theta12(i) = theta12(i-1) + error2(i) * (-gamma2) * dtheta12;
    theta22(i) = theta22(i-1) + (-gamma2) * error2(i) * dtheta22;
    
    % Calculate the control input u(t) and system output y(t) at the next time step
    Y2 =( (0.5*theta12(i))/(s-0.5*theta22(i)+1) )* UC;
    y_inv2 = ilaplace(Y2, s, t);
    y2(i) = double(subs(y_inv2, t, time(i)));
end

% Initialize arrays to store controller values and system outputs
gamma3 = 2;

theta13= zeros(size(time));
theta23 = zeros(size(time));
theta13(1) = 0;
theta23(1) = 0;

dtheta13 = zeros(size(time));
dtheta23 = zeros(size(time));
dtheta13(1) = 0;
dtheta23(1) = 0;

y3 = zeros(size(time));
y3(1) = 0; % Initial condition for y

% Initial conditions for theta1 and theta2


error3 = zeros(size(time));
error3(1) = 0;

% Simulate the system over time
for i = 2:length(time)
    % Calculate the current error
    error3(i) = y3(i-1) - ym_numerical(i-1);
    
    dtheta13 = (0.5/(s-0.5*theta23(i-1)+1)) * UC ;
    dtheta23 = ((0.5*0.5*theta13(i-1))/((s-0.5*theta23(i-1)+1)^2)) * UC ;

    dtheta13 = ilaplace(dtheta13, s, t);
    dtheta13 = double(subs(dtheta13, t, time(i)));
    
    dtheta23 = ilaplace(dtheta23, s, t);
    dtheta23 = double(subs(dtheta23, t, time(i)));

    % Update theta1 and theta2 using the MIT rule
    theta13(i) = theta13(i-1) + error3(i) * (-gamma3) * dtheta13;
    theta23(i) = theta23(i-1) + (-gamma3) * error3(i) * dtheta23;
    
    % Calculate the control input u(t) and system output y(t) at the next time step
    Y3 =( (0.5*theta13(i))/(s-0.5*theta23(i)+1) )* UC;
    y_inv3 = ilaplace(Y3, s, t);
    y3(i) = double(subs(y_inv3, t, time(i)));
end


% Plot theta2 vs. theta1
figure;
plot(theta11, theta21, 'm--', 'LineWidth', 1.5, 'DisplayName', 'gamma=0.5');
hold on;
plot(theta12, theta22, 'b:', 'LineWidth', 1.5, 'DisplayName', 'gamma=1');
plot(theta13, theta23, 'g-.', 'LineWidth', 1.5, 'DisplayName', 'gamma=2');
hold off;
xlabel('\theta_1');
ylabel('\theta_2');
title('\theta_2 vs. \theta_1');
grid on;
legend('Location', 'best');

% Plot theta1 with time
figure;
line([time(1), time(end)], [4, 4], 'Color', 'red', 'LineWidth', 1.5 , 'DisplayName', 'Desired Theta1');
hold on;
plot(time, theta11, 'm--', 'LineWidth', 1.5, 'DisplayName', 'gamma=0.5 Theta1');
plot(time, theta12, 'b:', 'LineWidth', 1.5, 'DisplayName', 'gamma=1 Theta1');
plot(time, theta13, 'g-.', 'LineWidth', 1.5, 'DisplayName', 'gamma=2 Theta1');
hold off;
xlabel('Time (s)');
ylabel('theta1');
title('theta1 with Time');
grid on;
legend('Location', 'best');

% Plot theta2 with time
figure;
line([time(1), time(end)], [-2, -2], 'Color', 'red', 'LineWidth', 1.5 , 'DisplayName', 'Desired Theta2');
hold on;
plot(time, theta21, 'm--', 'LineWidth', 1.5, 'DisplayName', 'gamma=0.5 Theta2');
plot(time, theta22, 'b:', 'LineWidth', 1.5, 'DisplayName', 'gamma=1 Theta2');
plot(time, theta23, 'g-.', 'LineWidth', 1.5, 'DisplayName', 'gamma=2 Theta2');
hold off;
xlabel('Time (s)');
ylabel('theta2');
title('theta2 with Time');
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
xlim([0, 30]);
