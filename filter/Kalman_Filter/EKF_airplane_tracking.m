
    clc;
    clear;
    close all;

    % Simulation parameters
    dt = 0.1; % Time step
    T = 10; % Total time
    t = 0:dt:T;

    % True initial state [x, y, vx, vy]
    x_true = [0; 0; 5; 3];

    % State and measurement noise covariances
    % 잡음이 서로 상관성이 있을시엔 비대각 행렬로 만들서 사용
    Q = diag([0.1, 0.1, 0.1, 0.1]);
    R = diag([10, 10]);

    % Initial state estimate and covariance
    x_est = [0; 0; 0; 0];
    P = 10 * eye(4);

    % Arrays to store results
    x_true_history = zeros(4, length(t));
    x_est_history = zeros(4, length(t));

    for k = 1:length(t)
        % True state update
        a_true = [0.1; -0.05]; % True acceleration
        x_true = state_transition(x_true, a_true, dt) + sqrt(Q) * randn(4, 1);

        % Measurement update
        z = measurement_function(x_true) + sqrt(R) * randn(2, 1);

        % EKF prediction
        [x_pred, A] = state_transition(x_est, [0; 0], dt);
        P_pred = A * P * A' + Q;

        % EKF update
        H = measurement_jacobian(x_pred);
        K = P_pred * H' / (H * P_pred * H' + R);
        x_est = x_pred + K * (z - measurement_function(x_pred));
        P = (eye(4) - K * H) * P_pred;

        % Store results
        x_true_history(:, k) = x_true;
        x_est_history(:, k) = x_est;
    end

    figure;
    plot(x_true_history(1, :), x_true_history(2, :), 'g', 'LineWidth', 2);
    hold on;
    plot(x_est_history(1, :), x_est_history(2, :), 'r--', 'LineWidth', 2);
    legend('True Position', 'Estimated Position');
    xlabel('X Position');
    ylabel('Y Position');
    title('True vs Estimated Position');
    grid on;

    % State transition function
    function [x_next, A] = state_transition(x, u, dt)
        A = [1, 0, dt, 0;
             0, 1, 0, dt;
             0, 0, 1, 0;
             0, 0, 0, 1];
        B = [0.5 * dt^2, 0;
             0, 0.5 * dt^2;
             dt, 0;
             0, dt];
        x_next = A * x + B * u;
    end

    % Measurement function
    function z = measurement_function(x)
        H = [1, 0, 0, 0;
             0, 1, 0, 0];
        z = H * x;
    end

    % Measurement Jacobian
    function H = measurement_jacobian(~)
        H = [1, 0, 0, 0;
             0, 1, 0, 0];
    end
