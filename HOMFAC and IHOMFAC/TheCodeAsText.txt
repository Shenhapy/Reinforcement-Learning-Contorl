T=1000;

% System dynamics
y = 0.5*ones(T, 1);
u = 0.5*ones(T, 1);
y_ast = 0.5*ones(T, 1);

% High-order MFAC parameters
eta = 0.8;
lambda = 0.1;
mu = 0.01;
rho = 0.8;
l = 4;
alpha = [1/2, 1/4, 1/8, 1/8];
eps = 0.01;

% High-order MFAC algorithm
phi = 0.5*ones(T, 1);
phi_hat = 0.5*ones(T, 1);

for t=1:T
 % Reference trajectory
    if t <= 300
        y_ast(t) = 0.5 * (-1)^(round(t/100));
    elseif t > 300 && t <= 700
        y_ast(t) = 0.5 * sin(t*pi/100) + 0.3 * cos(t*pi/50);
    else
        y_ast(t) = 0.5 * (-1)^(round(t/100));
    end
end


y(1)=0.6;
u(l)=0.2;
for t=1:T

    if t > l
    delta_u = u(t-1) - u(t-2);
    delta_y = y(t) - y(t-1);
   % delta_y = y(t) - y(t-1) - phi(t-1) * delta_u;
    
    phi_hat(t) = phi_hat(t-1) + ((eta * delta_u) / (mu + delta_u^2)) * ...
                 (delta_y - phi_hat(t-1) * delta_u);
    
        % Reset condition
        if abs(phi_hat(t)) <= eps
            % phi_hat(t) = phi_hat(1);
        end
        if abs(delta_u) <= eps
            % phi_hat(t) = phi_hat(1);
        end
        if sign(phi_hat(t)) ~= sign(phi_hat(1))
            % phi_hat(t) = phi_hat(1);
        end
    phi(t) = phi_hat(t);

    sum_term = 0;
    for i = 1:l
        sum_term = sum_term + alpha(i) * u(t-i);
    end

    u(t) = (phi(t)^2 / (lambda + phi(t)^2)) * u(t-1) + ...
           (lambda / (lambda + phi(t)^2)) * sum_term + ...
           (rho * phi(t) / (lambda + phi(t)^2)) * (y_ast(t) - y(t));
    
    end
    %nonlinear system
    if t <= 500
        y(t+1) = y(t)/(1 + y(t)^2) + u(t)^3;
    else
        y(t+1) = (y(t)*y(t-1)*y(t-2)*u(t-1)*(y(t-2)-1) + round((t)/500)*u(t)) / ...
               (1 + y(t-1)^2 + y(t-2)^2);
    end

end


T=1000;

% System dynamics
y_improved = 0.5*ones(T, 1);
y_ast = 0.5*ones(T, 1);

% High-order MFAC parameters
eta = 0.8;
lambda = 0.1;
mu = 0.01;
rho = 0.8;
l = 4;
l_imp=6;
alpha = [1/2, 1/4, 1/8, 1/8];
beta = [1/2, 1/4, 1/8, 1/16, 1/32, 1/32];
eps = 0.01;

% High-order MFAC algorithm
phi_hat_improved = 0.5*ones(T, 1);
u_improved = 0.5*ones(T, 1);

for t=1:T
 % Reference trajectory
    if t <= 300
        y_ast(t) = 0.5 * (-1)^(round(t/100));
    elseif t > 300 && t <= 700
        y_ast(t) = 0.5 * sin(t*pi/100) + 0.3 * cos(t*pi/50);
    else
        y_ast(t) = 0.5 * (-1)^(round(t/100));
    end
end

x=0;
s=0;
z=0;
y_improved(1)=0.7;
u(l_imp) = 0.1;
for t=1:T

    if t > l_imp
    delta_u = u_improved(t-1) - u_improved(t-2);
    delta_y = y_improved(t) - y_improved(t-1);

    
    sum_term = 0;
    for i = 1:l_imp
        sum_term = sum_term + beta(i) * phi_hat_improved(t-i);
    end
    
    phi_hat_improved(t) = sum_term + ...
                          ((eta * delta_u) / (mu + delta_u^2)) * ...
                          (delta_y - delta_u * sum_term);
    
    % Reset condition
        if abs(phi_hat_improved(t)) <= eps
            phi_hat_improved(t) = phi_hat_improved(1);
            x=x+1;
        end
        if abs(delta_u) <= eps
            phi_hat_improved(t) = phi_hat_improved(1);
            s=s+1;
        end
        if sign(phi_hat_improved(t)) ~= sign(phi_hat_improved(1))
            phi_hat_improved(t) = phi_hat_improved(1);
            z=z+1;
        end


    sum_term1 = 0;
    for i = 1:l
        sum_term1 = sum_term1 + alpha(i) * u_improved(t-i);
    end
    u_improved(t) = (phi_hat_improved(t)^2 / (lambda + phi_hat_improved(t)^2)) * u_improved(t-1) + ...
                    (lambda / (lambda + phi_hat_improved(t)^2)) * sum_term1 + ...
                    (rho * phi_hat_improved(t) / (lambda + phi_hat_improved(t)^2)) * (y_ast(t) - y_improved(t));
    end

   %nonlinear system
    if t <= 500
        y_improved(t+1) = y_improved(t)/(1 + y_improved(t)^2) + u_improved(t)^3;
    else
        y_improved(t+1) = (y_improved(t)*y_improved(t-1)*y_improved(t-2)*u_improved(t-1)*(y_improved(t-2)-1) + round((t)/500)*u_improved(t)) / ...
               (1 + y_improved(t-1)^2 + y_improved(t-2)^2);
    end

end


% Plot y, y_ast, and y_improved
figure;
plot(1:T, y_ast, 'k-', 'LineWidth', 1.5);
hold on;
plot(1:T+1, y, 'b--', 'LineWidth', 1.5);
plot(1:T+1, y_improved, 'm:', 'LineWidth', 1.5);
title('Comparison of Desired Input (y), Using High-Order MFAC, and Improved High-Order MFAC');
xlabel('Time');
ylabel('Tracking Performance');
legend('y(t)', 'High-Order MFAC', 'Improved High-Order MFAC');
hold off;


% Set the desired interval for zooming
zoom_start = 785;
zoom_end = 820;
% Plot y, y_ast, and y_improved
figure;
plot(1:T, y_ast, 'k-', 'LineWidth', 1.5);
hold on;
plot(1:T+1, y, 'b--', 'LineWidth', 1.5);
plot(1:T+1, y_improved, 'm:', 'LineWidth', 1.5);
title('The local tracking performance t: 785-820 of Desired Input (y), Using High-Order MFAC, and Improved High-Order MFAC');
xlabel('Time');
ylabel('Tracking Performance');
legend('y(t)', 'High-Order MFAC', 'Improved High-Order MFAC');
xlim([zoom_start, zoom_end]); % Set the x-axis limits
hold off;

% Set the desired interval for zooming
zoom_start = 920;
zoom_end = 1000;
% Plot y, y_ast, and y_improved
figure;
plot(1:T, y_ast, 'k-', 'LineWidth', 1.5);
hold on;
plot(1:T+1, y, 'b--', 'LineWidth', 1.5);
plot(1:T+1, y_improved, 'm:', 'LineWidth', 1.5);
title('The local tracking performance t: 920-1000 of Desired Input (y), Using High-Order MFAC, and Improved High-Order MFAC');
xlabel('Time');
ylabel('Tracking Performance');
legend('y(t)', 'High-Order MFAC', 'Improved High-Order MFAC');
xlim([zoom_start, zoom_end]); % Set the x-axis limits
hold off;

% Plot y, y_ast, and y_improved
figure;
plot(1:T, phi_hat, 'r-', 'LineWidth', 1.5);
hold on;
plot(1:T, phi_hat_improved, 'b--', 'LineWidth', 1.5);
title('PPD estimate value for High-Order MFAC, and Improved High-Order MFAC');
xlabel('Time');
ylabel('PPD Estimate Value');
legend('High-Order MFAC', 'Improved High-Order MFAC');

% Plot y and y_improved
figure;
plot(1:T, u, 'b--', 'LineWidth', 1.5);
hold on;
plot(1:T, u_improved, 'r:', 'LineWidth', 1.5);
title('Control Input for High-Order MFAC, and Improved High-Order MFAC');
xlabel('Time');
ylabel('Control Input u');
legend('High-Order MFAC', 'Improved High-Order MFAC');
hold off;