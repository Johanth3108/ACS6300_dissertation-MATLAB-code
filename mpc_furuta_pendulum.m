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

% Define MPC parameters
Ts = 0.1;   % Sampling time (seconds)
p = 10;     % Prediction horizon
m = 2;      % Control horizon

% Create a continuous-time state-space system model
sys = ss(A, B, C, D);

% Convert the system to discrete-time for MPC design
sys_d = c2d(sys, Ts);

% Extract discrete-time matrices
A = sys_d.A;
B = sys_d.B;
C = sys_d.C;
D = sys_d.D;

% Define MPC weights for performance tuning
weights = struct( ...
    'ManipulatedVariables', 0.5, ...      % Weight on control effort
    'ManipulatedVariablesRate', 0.1, ... % Weight on rate of change of control
    'OutputVariables', [5, 35, 1, 1], ...% Weights on output tracking
    'ECR', 1e5);                         % Weight on constraint violation

% Initialize the MPC controller
mpc_obj = mpc(sys, Ts, p, m, weights);

% Set constraints on manipulated variables (control input limits)
mpc_obj.ManipulatedVariables.Min = -10;
mpc_obj.ManipulatedVariables.Max = 10;

% Set constraints on output variables (e.g., pendulum angle limits)
mpc_obj.OutputVariables(2).Min = deg2rad(-2);  % Minimum pendulum angle
mpc_obj.OutputVariables(2).Max = deg2rad(2);   % Maximum pendulum angle

% Define simulation parameters
x0 = [0; 0; 0; 0];  % Initial state
Tfinal = 50;        % Total simulation time (seconds)
time = 0:Ts:Tfinal; % Time vector
num_steps = length(time);

% Preallocate arrays for simulation results
X = zeros(num_steps, length(x0));  % State trajectory
U = zeros(num_steps, 1);           % Control input trajectory
reference = zeros(num_steps, length(x0)); % Reference trajectory
cus_cost = zeros(num_steps, 1);    % Custom cost function values

% Initialize state and MPC controller state
x = x0;
mpc_state = mpcstate(mpc_obj);

% Main simulation loop
for i = 1:num_steps
    % Store current state and control input
    X(i, :) = x';
    U(i) = u;

    % Compute the measured output
    y = C * x;

    % Define reference trajectory
    if i < 50
        r = [0; 0; 0; 0];  % Equilibrium reference
    else
        % Periodic reference with phases
        phase = mod(floor((i - 50) / 50), 3);
        if phase == 0
            r = [pi/4; 0; 0; 0];  % Positive amplitude
        elseif phase == 1
            r = [0; 0; 0; 0];     % Zero amplitude
        else
            r = [-pi/4; 0; 0; 0]; % Negative amplitude
        end
    end

    % Compute control input using the MPC controller
    [u, Info] = mpcmove(mpc_obj, mpc_state, y, r);

    % Update system state
    x = A * x + B * u;

    % Store reference and custom cost
    reference(i, :) = r;
    cus_cost(i) = (r - x)' * diag([5, 35, 1, 1]) * (r - x) + 0.5 * u^2;
end

% Display total custom cost
disp('Total Custom Cost:');
disp(sum(cus_cost) * Ts);

% Plot results
figure;
subplot(3, 1, 1);
plot(time, rad2deg(X(:, 2)), 'm', 'LineWidth', 1.5);
hold on;
plot(time, rad2deg(reference(:, 2)), 'g--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\alpha (degrees)');
legend('Actual \alpha', 'Reference');
title('Pendulum Angle \alpha');
grid on;

subplot(3, 1, 2);
plot(time, rad2deg(X(:, 1)), 'r', 'LineWidth', 1.5);
hold on;
plot(time, rad2deg(reference(:, 1)), 'g--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\theta (degrees)');
legend('Actual \theta', 'Reference');
title('Arm Angle \theta');
grid on;

subplot(3,1,3);
plot(time, cus_cost, 'k', 'LineWidth', 1.5);
title('Optimal Cost');
xlabel('Time (s)');
ylabel('Cost');
legend('Instantaneous cost');
grid on;

sgtitle('MPC Simulation Results for Furuta Pendulum');

% Export variables for further analysis
timempc = time;
Xmpc_with = X;
cus_cost_with = cus_cost;
refmpc_with = reference;

% Uncomment below for exporting data without constraints if needed
% Xmpc_without = X;
% cus_cost_without = cus_cost;
% refmpc_without = reference;
