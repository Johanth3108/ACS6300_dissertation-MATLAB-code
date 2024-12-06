% Plot the simulation results
figure;
subplot(2,1,1);
plot(timempc, rad2deg(Xmpc(:, 2)), 'm', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\alpha (degrees)');
legend('Actual pendulum angle \alpha');
title('Actual Pendulum Angle \alpha');
hold on;
plot(tlqr, -1*(xlqr(:,2)), 'r', 'LineWidth', 1.5);
plot(timempc, rad2deg(refmpc(:, 2)), 'g--', 'LineWidth', 1.5);
legend('MPC controller output', 'LQR controller output', 'reference');
grid on;

subplot(2,1,2);
plot(timempc, rad2deg(Xmpc(:, 1)), 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\theta (degrees)');
legend('Arm angle \theta');
title('Arm Angle \theta');
hold on;
plot(tlqr, xlqr(:,1), 'k', 'LineWidth', 1.5);
plot(timempc, rad2deg(refmpc(:, 1)), 'g--', 'LineWidth', 1.5);
legend('MPC controller output', 'LQR controller output', 'reference');
grid on;

