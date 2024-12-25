% Plot the simulation results
figure;
subplot(3,1,1);
plot(timempc, rad2deg(Xmpc_without(:, 2)), 'b', 'LineWidth', 4);
xlabel('Time (s)');
ylabel('\alpha (degrees)');
legend('Actual pendulum angle \alpha');
title('Actual Pendulum Angle \alpha');
hold on;
plot(timempc, rad2deg(Xmpc_with(:, 2)), 'm', 'LineWidth', 4);
plot(timempc, rad2deg(refmpc_without(:, 2)), 'g--', 'LineWidth', 4);
legend('MPC without constraints', 'MPC  with constraints', 'reference');
grid on;

subplot(3,1,2);
plot(timempc, rad2deg(Xmpc_without(:, 1)), 'b', 'LineWidth', 4);
xlabel('Time (s)');
ylabel('\theta (degrees)');
legend('Arm angle \theta');
title('Arm Angle \theta');
hold on;
plot(timempc, rad2deg(Xmpc_with(:, 1)), 'm', 'LineWidth', 4);
plot(timempc, rad2deg(refmpc_with(:, 1)), 'g--', 'LineWidth', 4);
legend('MPC without constraints', 'MPC  with constraints', 'reference');
grid on;
sgtitle('Comparison of MPC Simulation Results with and without constraints');

subplot(3,1,3);
plot(timempc, cus_cost_without, 'b', 'LineWidth', 4);
title('Optimal Cost');
xlabel('Time (s)');
ylabel('Cost');
legend('Instantaneous cost');
hold on;
plot(timempc, cus_cost_with, 'm', 'LineWidth', 4);
legend('MPC cost without constraints', 'MPC cost with constraints');
grid on;
sgtitle('Comparison of MPC Simulation Results with and without constraints');
