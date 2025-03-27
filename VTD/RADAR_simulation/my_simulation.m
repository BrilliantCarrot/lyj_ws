% 레이더 가시성 테스트 메인 코드
%% 초기화
clear; clc; close all;
load C:/Users/leeyj/lab_ws/data/VTD/RADAR/output_map.mat;
X = MAP.X; % X 좌표
Y = MAP.Y; % Y 좌표
Z = MAP.alt; % 고도
x_min = 0; x_max = 30000;
y_min = 0; y_max = 40000;
% X와 Y 범위에 해당하는 인덱스 계산
x_idx = (X(1, :) >= x_min) & (X(1, :) <= x_max);
y_idx = (Y(:, 1) >= y_min) & (Y(:, 1) <= y_max);
X = double(X(y_idx, x_idx));
Y = double(Y(y_idx, x_idx));
Z = double(Z(y_idx, x_idx));
dx = 10;
dy = 10;
% 자른 간격으로 지형 단순화
X_reduced = X(1:dy:end, 1:dx:end); % X 데이터 축소
Y_reduced = Y(1:dy:end, 1:dx:end); % Y 데이터 축소
Z = Z(1:dy:end, 1:dx:end); % Z 데이터 축소
% 정규화 및 Y 좌표 방향 수정
X = X_reduced - min(min(X_reduced)); % X 좌표를 0부터 시작
Y = Y_reduced - min(min(Y_reduced)); % Y 좌표를 0부터 시작
% 레이더 설정
load C:/Users/leeyj/lab_ws/data/VTD/RADAR/Results_2GHz.mat
RADAR.RCS1 = Sth;
RADAR.theta = theta;
RADAR.psi = psi;
load C:/Users/leeyj/lab_ws/data/VTD/RADAR/Results_8GHz.mat
RADAR.RCS2 = Sth;
RADAR.lambda = freq2wavelen(2 * 10^9); % 기본 2GHz 파라미터
RADAR.Pt = 14000;  % [W] Peak Power
RADAR.tau = 0.00009;  % [s] Pulse Width
RADAR.G = 34;  % [dBi] Antenna Gain
RADAR.Ts = 290;  % [K] System Temperature
RADAR.L = 8.17;  % [dB] Loss
RADAR.sigma_0 = 10^(-20/10);  % Clutter Scattering Coefficient
RADAR.theta_A = deg2rad(1);  % Azimuth Beamwidth
RADAR.theta_E = deg2rad(2);  % Elevation Beamwidth
RADAR.SL_rms = 10^(-20.10);  % RMS Sidelobe Level
RADAR.R_e = 6.371e6;  % Earth Radius (m)
RADAR.c = 3e8;  % Speed of Light (m/s)
RADAR.prf = 1000; % [Hz] Pulse repetition frequency
RADAR.Du = RADAR.tau * RADAR.prf;
rcs_table = RADAR.RCS1;
radar_1 = double([10000, 10000, 230]);  % 레이더1 위치
% radar_2 = [14000, 14000, 300];  % 레이더2 위치

load C:/Users/leeyj/lab_ws/data/VTD/RADAR/path_new.mat;
load C:/Users/leeyj/lab_ws/data/VTD/RADAR/path.mat;
load C:/Users/leeyj/lab_ws/data/VTD/RADAR/sir_data.mat;
load C:/Users/leeyj/lab_ws/data/VTD/RADAR/visibility_matrix_sky.mat;
%% 가시성까지 고려된 환경에서 PSO 테스트
clc;
radar_1 = [10000, 10000, 230]; % 단일 레이더의 경우
radars = [10000, 10000, 230]; % 복수의 레이더 경우
start_pos = [0, 0, 200];
% end_pos = [1780, 5180, 450];
end_pos = [25000,34000,80];
% path = PSO_SIR_Optimization(radar_1, start_pos, end_pos, X, Y, Z, RADAR);
[path, sir_data, sir_values, visibility_values] = PSO_visibility(sir_data, radars, start_pos, end_pos, X, Y, Z, RADAR,visibility_matrix);
%% path와 sir_data 길이 맞춤
num_waypoints = size(path, 1);
current_sir_data_length = length(sir_data);
if current_sir_data_length < num_waypoints
    last_sir_matrix = sir_data{end}; % sir_data의 마지막 데이터 복사
    for k = (current_sir_data_length+1):num_waypoints
        sir_data{k} = last_sir_matrix; % 부족한 부분에 동일 데이터 채우기
    end
end
%% 
radar_1 = [10000, 10000, 230];
load C:/Users/leeyj/lab_ws/data/VTD/RADAR/output_map.mat;
X = MAP.X; % X 좌표
Y = MAP.Y; % Y 좌표
Z = MAP.alt; % 고도
x_min = 0; x_max = 30000;
y_min = 0; y_max = 40000;
% X와 Y 범위에 해당하는 인덱스 계산
x_idx = (X(1, :) >= x_min) & (X(1, :) <= x_max);
y_idx = (Y(:, 1) >= y_min) & (Y(:, 1) <= y_max);
X = double(X(y_idx, x_idx));
Y = double(Y(y_idx, x_idx));
Z = double(Z(y_idx, x_idx));
dx = 10;
dy = 10;
% 자른 간격으로 지형 단순화
X_reduced = X(1:dy:end, 1:dx:end); % X 데이터 축소
Y_reduced = Y(1:dy:end, 1:dx:end); % Y 데이터 축소
Z = Z(1:dy:end, 1:dx:end); % Z 데이터 축소
% 정규화 및 Y 좌표 방향 수정
X = X_reduced - min(min(X_reduced)); % X 좌표를 0부터 시작
Y = Y_reduced - min(min(Y_reduced)); % Y 좌표를 0부터 시작
visualize_PSO_SIR_2(path, sir_data, sir_values, visibility_values, radar_1, X, Y, Z);
%% 시각화
figure;
clf;
set(gcf, 'Position', [150, 75, 1200, 750]); % [left, bottom, width, height]
s = surf(X/1000, Y/1000, Z, 'EdgeColor', 'k', 'LineWidth',1);
hold on;
plot3(radar_1(1)/1000, radar_1(2)/1000, radar_1(3), ...
      'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k', 'LineWidth', 2);
colormap('jet');
colorbar;
view(20, 85);
grid on;
alpha(s, 0.8);
title('3D Surface');
xlabel('X Coordinate (meters)');
ylabel('Y Coordinate (meters)');
zlabel('Altitude (meters)');
%% 레이더를 특정 위치에 고정시킨 후 전체 지형에 대해 SIR을 구하는 코드
clc; close all;
radar_1 = double(radar_1);
SIR_matrix = RADAR_loc_sim(radar_1, X, Y, Z, RADAR);
figure;
clf;
set(gcf, 'Position', [150, 75, 1200, 750]);
s = surf(X/1000, Y/1000, Z, SIR_matrix, 'EdgeColor', 'k', 'LineWidth',1);
hold on;
plot3(radar_1(1) / 1000, radar_1(2) / 1000, radar_1(3), ...
      'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k', 'LineWidth', 2);
colorbar;
colormap(jet);
view(-20, 85);
grid on;
alpha(s, 0.8);
clim([min(SIR_matrix(:)), max(SIR_matrix(:))]);
c = colorbar;
c.Label.String = 'RADAR Signal (SIR in dB)';
xlabel('X [km]');
ylabel('Y [km]');
zlabel('Altitude [m]');
title('SIR Distribution Over Terrain');
%% 가시성 테스트
% 전체 지형에 대한 가시성 결과 시각화
visibility_matrix = LOS_test_new(radar_1, X, Y, Z);

figure;
clf;
set(gcf, 'Position', [150, 75, 1200, 750]);
hold on;
surf(X, Y, Z, visibility_matrix, 'EdgeColor', 'k', 'LineWidth', 1, 'FaceAlpha', 0.5);
colormap([1 0 0;0 1 0]);
colorbar;
view(-20, 85);
scatter3(radar_1(1), radar_1(2), radar_1(3), 50, 'k', 'filled');
title('LOS Visibility of RADAR');
xlabel('X [km]');
ylabel('Y [km]');
zlabel('Altitude (meters)');
legend('Terrain', 'Radar');
grid on;
%%
% 가시성이 없는 영역을 NaN으로 설정하여 회색으로 표현
SIR_display = SIR_matrix;
SIR_display(visibility_matrix == 0) = NaN;
figure;
clf;
set(gcf, 'Position', [150, 75, 1200, 750]);
hold on;
% 가시성 있는 영역의 SIR 값 표시
s = surf(X, Y, Z, SIR_display, 'EdgeColor', 'k', 'LineWidth', 1); % 가시성 영역에 대해 색상 적용
colormap('jet');
colorbar;
clim([min(SIR_matrix(:)), max(SIR_matrix(:))]);
% 가시성이 없는 영역을 회색으로 표시
gray_mask = surf(X, Y, Z, 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none'); 
set(gray_mask, 'FaceAlpha', 0.6); % 투명도 설정하여 구분 가능하도록 함
scatter3(radar_1(1), radar_1(2), radar_1(3), 100, 'k', 'filled');
view(-20, 85);
grid on;
title('SIR Visualization with LOS Constraint');
xlabel('X [km]');
ylabel('Y [km]');
zlabel('Altitude (meters)');
legend('Visible Terrain SIR', 'Non-visible Terrain (Gray)', 'Radar');
%% 특정 지역의 가시성 결과 시각화
target_1 = double([14922, 21354, 122]);
LOS_test_single(radar_1,target_1,X,Y,Z);
%% PSO 
% clc;
% radar_1 = [10000, 10000, 230]; % 단일 레이더의 경우
% radars = [10000, 10000, 230]; % 복수의 레이더 경우
% start_pos = [0, 0, 200];
% % end_pos = [1780, 5180, 450];
% end_pos = [25000,34000,80];
% % path = PSO_SIR_Optimization(radar_1, start_pos, end_pos, X, Y, Z, RADAR);
% [path, sir_data] = PSO_SIR_Optimization(radars, start_pos, end_pos, X, Y, Z, RADAR);
%% 가시성까지 고려된 환경에서 PSO 테스트
clc;
radar_1 = [10000, 10000, 230]; % 단일 레이더의 경우
radars = [10000, 10000, 230]; % 복수의 레이더 경우
start_pos = [0, 0, 200];
% end_pos = [1780, 5180, 450];
end_pos = [25000,34000,80];
% path = PSO_SIR_Optimization(radar_1, start_pos, end_pos, X, Y, Z, RADAR);
[path, sir_data, sir_values] = PSO_visibility(sir_data, radars, start_pos, end_pos, X, Y, Z, RADAR,visibility_matrix);
%%
visualize_PSO_SIR(path, sir_data, radar_1, X, Y, Z);

%% 추출된 path를 잇는 코드

num_points = 500; % 원하는 보간 후 총 좌표 개수
x_original = path(:, 1);
y_original = path(:, 2);
z_original = path(:, 3);
distances = [0; cumsum(sqrt(diff(x_original).^2 + diff(y_original).^2 + diff(z_original).^2))];
distance_query = linspace(0, distances(end), num_points);
x_interp = interp1(distances, x_original, distance_query, 'spline');
y_interp = interp1(distances, y_original, distance_query, 'spline');
z_interp = interp1(distances, z_original, distance_query, 'spline');
figure;
plot3(x_original, y_original, z_original, 'ro-', 'MarkerFaceColor', 'r', 'DisplayName', 'Original Waypoints');
hold on;
plot3(x_interp, y_interp, z_interp, 'b.-', 'DisplayName', 'Interpolated Path');
grid on;
legend;
xlabel('X Coordinate (meters)');
ylabel('Y Coordinate (meters)');
zlabel('Altitude (meters)');
title('Interpolated Flight Path Visualization');
path_new = [x_interp', y_interp', z_interp'];
% save('path_new.mat', 'path_new');
waypoints = path_new;
%%

waypoints = path;
n_waypoints = size(waypoints, 1);

% Simulation parameters
dt = 0.1;  % Time step (s)
T_final = 100; % Total simulation time (s)
g = 9.81; % Gravity (m/s^2)

% Initial aircraft state
pos = waypoints(1, :);  % Initial position (x, y, z)
vel = [50, 0, 0];        % Initial velocity [m/s] (assuming forward motion)
angles = [0, 0, 0];     % Euler angles [phi (roll), theta (pitch), psi (yaw)]
omega = [0, 0, 0];      % Angular velocity [rad/s]

% Inertia matrix (simplified aircraft model)
I_body = diag([5000, 6000, 7000]);

% Simulation loop
state_log = [];
time = 0;
wp_idx = 2; % Start from second waypoint

while time < T_final && wp_idx <= n_waypoints
    % Current waypoint target
    target = waypoints(wp_idx, :);
    
    % Compute desired direction
    direction = target - pos;
    dist = norm(direction);
    if dist < 50  % If close to waypoint, move to next
        wp_idx = wp_idx + 1;
        continue;
    end
    
    direction = direction / dist; % Normalize direction vector
    
    % Simple proportional guidance control for velocity
    desired_vel = direction * norm(vel);
    acc_cmd = (desired_vel - vel) / dt;
    
    % Update velocity and position
    vel = vel + acc_cmd * dt;
    pos = pos + vel * dt;
    
    % Compute desired yaw angle
    psi_desired = atan2(direction(2), direction(1));
    yaw_rate_cmd = (psi_desired - angles(3)) / dt;
    
    % Update angular velocity and angles (simplified dynamics)
    omega(3) = yaw_rate_cmd; % Only update yaw rate for now
    angles = angles + omega * dt;
    
    % Log state
    state_log = [state_log; time, pos, vel, angles];
    
    % Time update
    time = time + dt;
end

% Convert log to struct
sim_data.time = state_log(:, 1);
sim_data.pos = state_log(:, 2:4);
sim_data.vel = state_log(:, 5:7);
sim_data.angles = state_log(:, 8:10);

% Plot results
figure;
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'ro-', 'LineWidth', 2);
hold on;
plot3(sim_data.pos(:,1), sim_data.pos(:,2), sim_data.pos(:,3), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('6DoF Aircraft Path Following Simulation');
legend('Waypoints', 'Aircraft Path');
%% 무게 고려 6DoF

% 시뮬레이션 파라미터 및 초기 상태 변수 설정
dt = 0.1; % 시간 간격 (초)
Tmax = 2000; % 최대 시뮬레이션 시간 (초)
velocity = 100; % 초기 항공기 속도 (m/s)
N = 3; % PNG 비율 Gain
pos = waypoints(1, :); % 시작점
target_idx = 2; % 다음 웨이포인트 인덱스
quat = [1 0 0 0]; % 초기 쿼터니언 (롤, 피치, 요 회전 상태)
traj = [];
time = 0;

% 항공기 물리적 특성
mass = 1000; % 항공기 질량 (kg)
thrust = 5000; % 추력 (N)
gravity = [0 0 -9.81 * mass]; % 중력 가속도
Cd = 0.05; % 항력 계수
density = 1.225; % 공기 밀도 (kg/m^3)
A = 10; % 항공기 단면적 (m^2)

% 시뮬레이션 루프
while target_idx <= size(waypoints, 1) && time < Tmax
    target = waypoints(target_idx, :);
    rel_pos = target - pos;
    range = norm(rel_pos);
    direction = rel_pos / range; % 목표 방향 벡터
    
    % 목표에 도달하면 다음 웨이포인트로 변경
    if range < 50
        target_idx = target_idx + 1;
        if target_idx > size(waypoints, 1)
            break;
        end
        continue;
    end
    
    % 목표 웨이포인트를 향한 직선 추종
    velocity_vector = velocity * direction;
    
    % 공기 저항 계산
    speed = norm(velocity_vector);
    drag_force = 0.5 * density * speed^2 * A * Cd * (-velocity_vector / speed);
    
    % 총 가속도 계산 (추력, 공기저항, 중력 고려)
    force_total = thrust * direction + drag_force + gravity;
    acceleration = force_total / mass;
    
    % 속도 및 위치 업데이트
    vel = velocity_vector + acceleration * dt;
    pos = pos + vel * dt;
    yaw = atan2(vel(2), vel(1)); % 오일러 각도 업데이트 (기본적인 요(Psi) 회전 반영)
    pitch = atan2(-vel(3), sqrt(vel(1)^2 + vel(2)^2));
    roll = 0; % 간단한 모델에서 롤은 유지
    quat = eul2quat([yaw pitch roll]); % 쿼터니언 업데이트
    
    traj = [traj; pos];
    time = time + dt;
end

figure;
hold on;
plot3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 'ro-', 'LineWidth', 1); % 원래 웨이포인트
plot3(traj(:, 1), traj(:, 2), traj(:, 3), 'b-', 'LineWidth', 2); % 비행 궤적
scatter3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 1, 'r', 'filled');
scatter3(traj(end, 1), traj(end, 2), traj(end, 3), 80, 'g', 'filled'); % 최종 위치
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Altitude (m)');
title('6-DOF Flight Simulation with Direct Waypoint Following');
grid on;
view(3);
legend('Waypoints', 'Flight Path', 'Start/End Points');



%% 이전 6DoF - PNG(TPN)
dt = 0.1; % 시간 간격 (초)
Tmax = 2000; % 최대 시뮬레이션 시간 (초)
velocity = 100; % 항공기 속도 (m/s)
N = 3; % PNG 비율 Gain
pos = waypoints(1, :); % 시작점
target_idx = 2; % 다음 웨이포인트 인덱스
quat = [1 0 0 0]; % 초기 쿼터니언 (롤, 피치, 요 회전 상태)
traj = [];
time = 0; 
while target_idx <= size(waypoints, 1) && time < Tmax
    target = waypoints(target_idx, :);
    rel_pos = target - pos;
    range = norm(rel_pos);
    if range < 50
        target_idx = target_idx + 1;
        if target_idx > size(waypoints, 1)
            break;
        end
        continue;
    end
    % PNG 유도 - 선속도를 유지하며 조향 각도 결정
    LOS_rate = cross([0 0 velocity], rel_pos) / range^2; % LOS 변화율
    acc_cmd = N * cross(LOS_rate, [0 0 velocity]); % 유도 가속도
    vel = velocity * rel_pos / range + acc_cmd * dt;
    pos = pos + vel * dt;
    yaw = atan2(vel(2), vel(1)); % 오일러 각도 업데이트 (기본적인 요(Psi) 회전 반영)
    pitch = atan2(-vel(3), sqrt(vel(1)^2 + vel(2)^2));
    roll = 0; % 간단한 모델에서 롤은 유지
    quat = eul2quat([yaw pitch roll]); % 쿼터니언 업데이트
    traj = [traj; pos];
    time = time + dt;
end
figure;
hold on;
plot3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 'ro-', 'LineWidth', 1); % 원래 웨이포인트
plot3(traj(:, 1), traj(:, 2), traj(:, 3), 'b-', 'LineWidth', 2); % 비행 궤적
scatter3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 1, 'r', 'filled');
scatter3(traj(end, 1), traj(end, 2), traj(end, 3), 80, 'g', 'filled'); % 최종 위치
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Altitude (m)');
title('6-DOF Flight Simulation with PNG');
grid on;
view(3);
legend('Waypoints', 'Flight Path', 'Start/End Points');