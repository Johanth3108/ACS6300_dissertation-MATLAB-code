% Define the state-space matrices for the system
A = [0, 1, 0, 0;
     37.377, -0.515, 0, 0.142;
     0, 0, 0, 1;
     -8.228, 0.113, 0, -0.173];  % State matrix

B = [0;
     -35.42;
     0;
     43.28];  % Input matrix

C = eye(4);   % Output matrix
D = zeros(4,1);  % Feedforward matrix

% Define the weight matrices for the LQR controller
Q = diag([200, 1, 10, 1]);  % State weighting matrix
R = 50;                    % Control weighting scalar


%% Define the state-space matrices for the Furuta pendulum system
% State matrix (A), input matrix (B), output matrix (C), and feedforward matrix (D)

% Define the state-space matrices for the system
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, 39.32, -14.52, 0;
     0, 81.78, -13.98, 0];  % State matrix

B = [0;
     0;
     25.54;
     24.59];  % Input matrix

C = eye(4);   % Output matrix
D = zeros(4,1);  % Feedforward matrix

% Define the weight matrices for the LQR controller
Q = diag([10, 200, 1, 1]);  % State weighting matrix
R = 50;                    % Control weighting scalar

% Define the weight matrices for the LQR controller
% The weights penalize deviations from the desired states and control effort.
Q = diag([3, 8, 1, 1]);  % Penalizes state errors
R = 1;                   % Penalizes control effort

% Solve the continuous-time algebraic Riccati equation (ARE)
% This finds the optimal gain matrix to minimize the cost function
[X, L, G] = icare(A, B, Q, R); % X: ARE solution, L: closed-loop poles, G: eigenvalues

% Compute the LQR gain matrix K
K = inv(R) * B' * X;

% Display the computed gain matrix K
disp('The LQR gain matrix K is:');
disp(K);

% Define simulation parameters
% Initial conditions: [theta (arm angle); theta_dot; alpha (pendulum angle); alpha_dot]
x0 = [0; 0; 0; 0];  % Initial state of te furuta pendulum

% Time vector for simulation (50 seconds, 0.01-second time step)
t = 0:0.01:50;  

% Initialize arrays to store state, control input, reference, and cost
x = zeros(length(t), 4);    % State trajectory
u = zeros(length(t), 1);    % Control input
ref = zeros(length(t), 1);  % reference for arm angle
refa = zeros(length(t), 1);  % reference for pendulum angle
cost = zeros(length(t), 1);  % Instantaneous cost

% Set the initial state
x(1, :) = x0';

% Simulation loop
for i = 2:length(t)
    % Define reference trajectory
    if t(i) < 5
        ref(i) = 0;  
    else
        % Generate reference based on phase of 5-second intervals
        phase = mod(floor((t(i) - 5) / 5), 3); 
        if phase == 0
            ref(i) = pi/4;  % Positive amplitude
        elseif phase == 1
            ref(i) = 0;     % Zero amplitude
        else
            ref(i) = -pi/4; % Negative amplitude
        end
    end
    
    % Compute the control input based on state feedback
    u(i) = -K * x(i-1, :)';
    
    % Update the state using Euler integration with disturbance added to alpha_dot
    dx = A * x(i-1, :)' + B * u(i);
    dx(1) = dx(1) + ref(i);  % Changing the reference for theta (arm angle)
    x(i, :) = x(i-1, :) + dx' * (t(i) - t(i-1));

    % Compute the instantaneous cost for the current state and control input
    cost(i) = x(i, :) * Q * x(i, :)' + u(i)^2 * R;
end

% Convert angles from radians to degrees for visualization
x(:,1) = rad2deg(x(:,1));  % Convert theta to degrees
x(:,2) = rad2deg(x(:,2));  % Convert alpha to degrees

% Compute the total cost by approximating the integral of the instantaneous cost
total_cost = sum(cost) * (t(2) - t(1));  % Use the time step as the integration interval

% Display the total cost of the LQR controller
disp(['Total cost J = ', num2str(total_cost)]);

% Plot the simulation results
figure;

% Plot the pendulum angle theta over time
subplot(3,1,1);
plot(t, -1*(x(:,2)), 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\alpha (degrees)');
title('Pendulum Angle \alpha');
%legend('Pendulum angle \alpha');
grid on;
hold on;
plot(t, rad2deg(refa), 'g--', 'LineWidth', 1.5);
legend('Pendulum angle \alpha', 'reference');
hold off;

subplot(3,1,2);
plot(t, x(:,1), 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\theta (degrees)');
title('Arm Angle \theta');
grid on;
hold on;
plot(t, rad2deg(ref), 'g--', 'LineWidth', 1.5);
legend('Arm angle \theta', 'reference');
hold off;

% Plot the control input over time
subplot(3,1,3);
plot(t, u*2, 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Input in Voltage u(t)');
title('Control Input');
legend('Control Input');
grid on;

% Plot the instantaneous cost over time
% subplot(3,1,3);
% plot(t, cost, 'k', 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Cost');
% title('Optimal Cost');
% legend('Optimal cost');
% grid on;

% Add a super title to the figure
sgtitle('LQR Simulation Results for Furuta Pendulum');

%% export variables for plotting
tlqr = t;
xlqr = x;
penreflqr = refa;
armreflqr = ref;
