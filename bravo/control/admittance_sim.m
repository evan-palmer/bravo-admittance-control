% Admittance Controller Simulation
% 06/01/2023
clear
clc

% Load Bravo arm
bravo = importrobot('bravo7.urdf', DataFormat='row');

% Controller Gains

% (YZ) ee-frame - WAS OK. FOUND AN IDEAL
% Md = diag([0, 5, 8.2, 0, 0, 0]);
% Kp = diag([0, 0.25, 4.5, 0, 0, 0]);
% Kd = diag([0, 0.001, 0.02, 0, 0, 0]);
% Kf = diag([0, 0.415, 0.1, 0, 0, 0]);
% B = diag([0, 1, 1, 0, 0, 0]);

% (XY) ee-frame - DIDN'T DO ANYTHING
% Md = diag([5, 5, 0, 0, 0, 0]);
% Kp = diag([1, 0.25, 0, 0, 0, 0]);
% Kd = diag([1, 0.001, 0, 0, 0, 0]);
% Kf = diag([1, 0.415, 0, 0, 0, 0]);
% B = diag([1, 1, 0, 0, 0, 0]);

% (XYZ) ee-frame - EHHH DIDN'T REALLY LIKE IT
% Md = diag([10, 100, 10, 0, 0, 0]);
% Kp = diag([10, 50, 10, 0, 0, 0]);
% Kd = diag([10, 10, 10, 0, 0, 0]);
% Kf = diag([10, 20, 10, 0, 0, 0]);
% B = diag([1, 1, 1, 0, 0, 0]);

% (Y) ee-frame - BEST OUTPUT (10, 0.1, 4, 0.4)
Md = diag([0, 10, 0, 0, 0, 0]);
Kp = diag([0, 0.1, 0, 0, 0, 0]);
Kd = diag([0, 4, 0, 0, 0, 0]);
Kf = diag([0, 0.4, 0, 0, 0, 0]);
B = diag([0, 1, 0, 0, 0, 0]);

% Simulation parameters
dt = 0.01;    % Time step (s)
t_end = 5;    % Simulation duration (s)
q0 = [pi, 3*pi/4, 3*pi/4, 0, 3.14, pi/3, 0]; % Starting config
num_joints = length(q0);

% Pre-allocate arrays for storing simulation results
t = 0:dt:t_end;
q = zeros(length(t), num_joints);
qdot = zeros(length(t), num_joints);
wrench = zeros(length(t), 6);
xe_pos = zeros(length(t), 3);
xe = zeros(length(t), 6);
pos_error = zeros(length(t), 6);
f_error = zeros(length(t), 6);

% Initial conditions
q(1, :) = q0; % Position (m)
qdot(1, :) = zeros(1, num_joints); % Velocity (m/s)

% Desired sensor readings
fd = zeros(1, 6);
xd_transform = getTransform(bravo, q0, 'ee_link', 'world');
xd_rot = xd_transform(1:3, 1:3);
xd_pos = xd_transform(1:3, 4)';
xd = [xd_pos, rotm2eul(xd_rot)];

% Simulate 25N over 3-seconds
fy = 0;
f_step = 0.17;
for i = 101:400
    wrench(i, :) = [0, fy, 0, 0, 0, 0];
    if i <= 250
        fy = fy + f_step;
    else
        fy = fy - f_step;
    end
end

% Add a reversal kick to bring ee-pos back to equilibrium
% f_step = 0.5;
% for i = 501:800
%     wrench(i, :) = [0, fy, 0, 0, 0, 0];
%     if i <= 650
%         fy = fy - f_step;
%     else
%         fy = fy + f_step;
%     end
% end

% Simulation loop
for i = 1:length(t)
    % Compute jacobian
    jc = geometricJacobian(bravo, q(i, :), 'ee_link');

    % End-effector position and orientation
    xe_transform = getTransform(bravo, q(i, :), 'ee_link', 'world');
    xe_rot = xe_transform(1:3, 1:3);
    xe_pos(i, :) = xe_transform(1:3, 4)';
    xe(i, :) = [xe_pos(i, :), rotm2eul(xe_rot)];

    % End-effector velocity
    ve = jc * qdot(i, :)';

    % End-effector error terms
    pos_error(i, :) = xd - xe(i, :);
    f_error(i, :) = fd' - B * wrench(i, :)';

    % Update joint velocity with control law
    % With ee-position error
    % qdot(i + 1, :) = pinv(jc) * pinv(Md) * (-Kd * ve + Kp * (pos_error(i, :) + (Kf * f_error(i, :)')')');
    % Without ee-position error
    qdot(i + 1, :) = pinv(jc) * pinv(Md) * (-Kd * ve + Kp * (Kf * f_error(i, :)'));

    % Clamp joint velocity
    for k = 1:length(qdot(i + 1, :))
        if qdot(i + 1, k) > 0.0833
            qdot(i + 1, k) = 0.0833;
        end
        if qdot(i + 1, k) < -0.0833
            qdot(i + 1, k) = -0.0833;
        end
    end

    % Integrate velocity to get joint positions
    q(i + 1, :) = q(i, :) + qdot(i + 1, :) * dt;

    % Clamp joint angle changes
    if q(i + 1, 2) > 3.14
        q(i + 1, 2) = 3.14;
    end
    if q(i + 1, 3) > 3.14
        q(i + 1, 3) = 3.14;
    end
    if q(i + 1, 5) > 3.14
        q(i + 1, 5) = 3.14;
    end

end
disp('finished simulation')

%% Plots
clf

% Plot 3D robot motion
for i = 1:length(t)
    % Display robot
    show(bravo, q(i, :), 'PreservePlot', false, 'FastUpdate', true);
    xlim([-0.1, 0.9])
    ylim([-0.1, 0.1])
    zlim([-0.1, 0.6])
    drawnow
end

figure
subplot(3, 1, 1)
plot(t, q(1:end-1, 1)')
hold on
plot(t, q(1:end-1, 2)')
hold on
plot(t, q(1:end-1, 3)')
hold on
plot(t, q(1:end-1, 4)')
hold on
plot(t, q(1:end-1, 5)')
hold on
plot(t, q(1:end-1, 6)')
hold on
plot(t, q(1:end-1, 7)')
legend('j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7')
xlabel('time (s)')
ylabel('joint angle (rads)')
title('Joint angles')

subplot(3, 1, 2)
% plot(t, xe(:, 1)')
% hold on
% plot(t, xe(:, 2)')
% hold on
plot(t, xe(:, 3)')
legend('z')
xlabel('time (s)')
ylabel('position (m)')
title('End-effector position (world frame)')

subplot(3, 1, 3)
plot(t, wrench(:, 2)')
xlabel('time (s)')
ylabel('force (N)')
title('End-effector force')
