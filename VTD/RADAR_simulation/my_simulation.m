% 레이더 가시성 테스트 메인 코드
%% 간단한 지형 생성을 통해 알고리즘 검증
% 지형 생성 방법 1, 안 씀 
% grid_size = 3601;
% dx = 10; dy = 10; % 셀 간격 (임의의 단위, 예: meter)
% [X, Y] = meshgrid(0:dx:(grid_size-1)*dx, 0:dx:(grid_size-1)*dx);
% Z = zeros(grid_size, grid_size, 'int16');
% 
% % center = round(grid_size / 2);
% % width = 200;  % 언덕의 반지름 크기
% % for i = -width:width
% %     for j = -width:width
% %         if sqrt(i^2 + j^2) <= width
% %             Z(center + i, center + j) = int16(500);
% %         end
% %     end
% % end
% 
% % 가우시안 형태 고도 분포 언덕 생성
% center_x = X(1, round(grid_size/2));
% center_y = Y(round(grid_size/2), 1);
% hill_radius = 6000;  % 언덕 반지름 (실제 거리 단위)
% hill_height = 500;   % 언덕 최고 높이
% sigma = hill_radius / 2;  % 표준편차는 반지름의 절반 정도
% Z = hill_height * exp(-(((X - center_x).^2 + (Y - center_y).^2) / (2 * sigma^2)));
% 
% MAP2.X = X;
% MAP2.Y = Y;
% MAP2.alt = Z;
% X = MAP2.X; % X 좌표
% Y = MAP2.Y; % Y 좌표
% Z = double(MAP2.alt); % 고도
% X_plot = X(1:dy:end, 1:dx:end);
% Y_plot = Y(1:dy:end, 1:dx:end);
% Z_plot = Z(1:dy:end, 1:dx:end);
% save('MAP2.mat', 'MAP2');
% load C:/Users/leeyj/lab_ws/data/VTD/RADAR/Results_2GHz.mat
% RADAR.RCS1 = Sth;
% RADAR.theta = theta;
% RADAR.psi = psi;
% load C:/Users/leeyj/lab_ws/data/VTD/RADAR/Results_8GHz.mat
% RADAR.RCS2 = Sth;
% RADAR.lambda = freq2wavelen(8 * 10^9); % 기본 8GHz 파라미터
% RADAR.Pt = 6000;  % [W] Peak Power
% RADAR.tau = 0.0001;  % [s] Pulse Width
% RADAR.G = 39;  % [dBi] Antenna Gain
% RADAR.Ts = 290;  % [K] System Temperature
% RADAR.L = 8.17;  % [dB] Loss
% RADAR.sigma_0 = 10^(-20/10);  % Clutter Scattering Coefficient
% RADAR.theta_A = deg2rad(1);  % Azimuth Beamwidth
% RADAR.theta_E = deg2rad(2);  % Elevation Beamwidth
% RADAR.SL_rms = 10^(-20.10);  % RMS Sidelobe Level
% RADAR.R_e = 6.371e6;  % Earth Radius (m)
% RADAR.c = 3e8;  % Speed of Light (m/s)
% RADAR.prf = 1000; % [Hz] Pulse repetition frequency
% RADAR.Du = RADAR.tau * RADAR.prf;
% rcs_table = RADAR.RCS1;
% radar_1 = double([12700, 31240, 20]);
% radars = [12700, 31240, 20];
% start_pos = [29300, 30000, 30];
% end_pos = [8800, 7110, 30];
% figure;
% clf;
% set(gcf, 'Position', [150, 75, 1200, 750]);
% % s = surf(MAP2.X / 1000, MAP2.Y / 1000, Z, Z, 'EdgeColor', 'none');
% s = surf(X_plot / 1000, Y_plot / 1000, Z_plot, Z_plot, 'EdgeColor', 'none');
% hold on;
% plot3(radar_1(1)/1000, radar_1(2)/1000, radar_1(3), ...
%       'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k', 'LineWidth', 2);
% colormap('jet');
% colorbar;
% shading interp
% view(20, 85);
% grid on;
% alpha(s, 0.9);
% title('3D Surface');
% xlabel('X Coordinate (km)');
% ylabel('Y Coordinate (km)');
% zlabel('Altitude (m)');
% clim([min(Z(:)), max(Z(:))]);

%% 지형 생성 방법 2, 안 씀

% grid_size = 3601;
% x_max = 72435.4692673415;
% y_max = 111339.274863030;
% dx = x_max / (grid_size - 1);
% dy = y_max / (grid_size - 1);
% [X, Y] = meshgrid(0:dx:x_max, 0:dy:y_max);
% Z = zeros(grid_size, grid_size, 'int16');
% center = round(grid_size / 2);
% radius = 200;
% for i = -radius:radius
%     for j = -radius:radius
%         if sqrt(i^2 + j^2) <= radius
%             Z(center+i, center+j) = int16(300);
%         end
%     end
% end
% MAP3.X = X;
% MAP3.Y = Y;
% MAP3.alt = Z;
% save('MAP3.mat', 'MAP3');

%% 초기화

clear; close all;
% load C:/Users/leeyj/lab_ws/data/VTD/RADAR/DTED_mountain.mat;
% mountain = load ("C:/Users/leeyj/lab_ws/data/VTD/RADAR/DTED_mountain.mat");
load C:/Users/leeyj/lab_ws/data/VTD/RADAR/map_flat_land.mat;
X = MAP.X; % X 좌표
Y = MAP.Y; % Y 좌표
Z = MAP.alt; % 고도
% 원래 방식으론 데이터 크기 문제로 인해 지형을 자른 특정 영역만 확인
% 산지의 경우 
% x_min = 0; x_max = 40000; y_min = 40000; y_max=max(Y(:));
% 평지의 경우(이전) 
x_min = 0; x_max = 30000; y_min = 0; y_max = 40000;
% 전체 영역
% x_min = min(X(:)); 
% x_max = max(X(:));
% y_min = min(Y(:)); 
% y_max = max(Y(:));
% X와 Y 범위에 해당하는 인덱스 계산
x_idx = (X(1, :) >= x_min) & (X(1, :) <= x_max);
y_idx = (Y(:, 1) >= y_min) & (Y(:, 1) <= y_max);
X = double(X(y_idx, x_idx));
Y = double(Y(y_idx, x_idx));
Z = double(Z(y_idx, x_idx));
% 범위 제한 지형에선 dx, dy 10 설정
dx = 10; dy = 10;
% 자른 간격으로 지형 단순화
X_reduced = X(1:dy:end, 1:dx:end); % X 데이터 축소
Y_reduced = Y(1:dy:end, 1:dx:end); % Y 데이터 축소
Z = Z(1:dy:end, 1:dx:end); % Z 데이터 축소
% 정규화 및 좌표 방향 수정
X = X_reduced - min(min(X_reduced)); % X 좌표를 0부터 시작
Y = Y_reduced - min(min(Y_reduced)); % Y 좌표를 0부터 시작
% 레이더 설정
load C:/Users/leeyj/lab_ws/data/VTD/RADAR/Results_2GHz.mat
RADAR.RCS1 = Sth;
RADAR.theta = theta;
RADAR.psi = psi;
load C:/Users/leeyj/lab_ws/data/VTD/RADAR/Results_8GHz.mat
RADAR.RCS2 = Sth;
RADAR.lambda = freq2wavelen(8 * 10^9); % 기본 8GHz 파라미터
RADAR.Pt = 6000;  % [W] Peak Power
RADAR.tau = 0.0001;  % [s] Pulse Width
RADAR.G = 39;  % [dBi] Antenna Gain
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
% radar_1 = double([45000, 60000, 750]); % 레이더 위치(산지)
% radar_1 = double([10000, 10000, 230]); % 레이더 위치(이전 평지에서)
% radar_1 = double([40000, 60000, 130]); % 레이더 위치(새 평지(simple terrain)에서)
% radar_1 = double([18000, 24000, 20]); % 레이더 위치(very simple terrain에서)
% radar_2 = [14000, 14000, 300];  % 레이더2 위치

% load C:/Users/leeyj/lab_ws/data/VTD/RADAR/path_new.mat;
% load C:/Users/leeyj/lab_ws/data/VTD/RADAR/path.mat;
% load C:/Users/leeyj/lab_ws/data/VTD/RADAR/SIR_matrix.mat;
% load C:/Users/leeyj/lab_ws/data/VTD/RADAR/visibility_matrix.mat;

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

%% 간단한 지형 생성
% 위 코드 실행 먼저하고 실행

grid_size = 181; % 시뮬레이션 환경 크기
% Z = zeros(181, 181, 'double');
visibility_matrix = ones(181, 181, 'double');
% 언덕 생성
% center_x = X(1, round(grid_size/2));
% center_y = Y(round(grid_size/2), 1);
% center_x = 15000; center_y = 15000;
% hill_radius = 6000;  % 언덕 반지름 (실제 거리 단위)
% hill_height = 300;   % 언덕 최고 높이
% sigma = hill_radius / 2; % 가우시안 표준 편차
% 완만한 언덕
% Z = hill_height * exp(-(((X - center_x).^2 + (Y - center_y).^2) / (2 * sigma^2)));
% 원뿔 모양
% distance = sqrt((X - center_x).^2 + (Y - center_y).^2);
% Z(distance <= hill_radius) = hill_height * ...
%     (1 - distance(distance <= hill_radius) / hill_radius);

% very simple terrain에서 복수의 레이더 설치 위치
% radars = [
%     21000, 11000, 12;
%     25000, 22000, 12;
%     18000, 26000, 12;
%     11000, 5000, 12; 
%     12000, 18000, 12];
% start_pos = [28000, 12000, 30];
% end_pos = [3500, 17000, 30];

% very simple terrain에서 단일 레이더
% radars = double([10000, 12000, 12]);
% start_pos = [8300, 19000, 30];
% end_pos = [18000, 7700, 30];

% simple terrain에서 단일 레이더(가시성 항 검증)
% radars = double([6600, 7100, 12]);
% start_pos = [6900, 21000, 30];
% end_pos = [22000, 4900, 30];

figure;
clf;
set(gcf, 'Position', [150, 75, 1200, 750]);
s = surf(X/1000, Y/1000, Z, 'EdgeColor', 'k', 'LineWidth',1);
hold on;
for i = 1:size(radars, 1)
    plot3(radars(i,1)/1000, radars(i,2)/1000, radars(i,3),...
        'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k', 'LineWidth', 2);
end
plot3(start_pos(1)/1000, start_pos(2)/1000, start_pos(3),...
    'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'LineWidth', 2);
plot3(end_pos(1)/1000, end_pos(2)/1000, end_pos(3), ...
      'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 2);
colormap('jet');
% colorbar;
view(20, 85);
grid on;
alpha(s, 0.8);
title('Simplified Terrain with a Hill');
xlabel('X Coordinate [km]');
ylabel('Y Coordinate [km]');
zlabel('Altitude (meters)');

% radar_1 = double([22500, 81000, 20]);
% radars = double([22500, 81000, 20]);
% start_pos = [60000, 75000, 30];
% end_pos = [12000, 21000, 30];
% figure;
% clf;
% set(gcf, 'Position', [150, 75, 1200, 750]); % [left, bottom, width, height]
% s = surf(X/1000, Y/1000, Z, 'EdgeColor', 'k', 'LineWidth',1);
% hold on;
% plot3(radar_1(1)/1000, radar_1(2)/1000, radar_1(3), ...
%       'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k', 'LineWidth', 2);
% colormap('jet');
% colorbar;
% view(20, 85);
% grid on;
% alpha(s, 0.8);
% title('3D Surface');
% xlabel('X Coordinate [km]');
% ylabel('Y Coordinate [km]');
% zlabel('Altitude (meters)');

%% 전체 지형에 대해 SIR 계산 (SIR 계산 테스트)

SIR_matrix = RADAR_loc_sim(radars, X, Y, Z, RADAR);
figure;
clf;
set(gcf, 'Position', [150, 75, 1200, 750]);
s = surf(X/1000, Y/1000, Z, SIR_matrix, 'EdgeColor', 'k', 'LineWidth',1);
colorbar;
colormap(jet);
hold on;
for i = 1:size(radars, 1)
    plot3(radars(i,1)/1000, radars(i,2)/1000, radars(i,3), ...
          'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k', 'LineWidth', 2);
end
plot3(start_pos(1)/1000, start_pos(2)/1000, start_pos(3),...
    'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'LineWidth', 2);
plot3(end_pos(1)/1000, end_pos(2)/1000, end_pos(3), ...
      'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 2);
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

%% 레이더와 비행체간 가시성 확인 및 가시성 행렬 산출

visibility_matrix = LOS_test_multi(radars, X, Y, Z);

%% 가시성 산출 결과 시각화

figure; clf;
set(gcf, 'Position', [150, 75, 1200, 750]);
hold on;
s = surf(X, Y, Z, visibility_matrix, 'EdgeColor', 'k', 'LineWidth', 1, 'FaceAlpha', 0.5);
h_start = plot3(start_pos(1), start_pos(2), start_pos(3), ...
      'ks', 'MarkerSize', 15, 'MarkerFaceColor', 'w');
h_end = plot3(end_pos(1), end_pos(2), end_pos(3), ...
      'ks', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
h_radar = scatter3(radars(1), radars(2), radars(3), 50, 'yellow', 'filled');
h_path = scatter3(path(:,1),path(:,2),path(:,3),60, 'k', 'filled');
colormap([0 1 0; 1 0 0]);
caxis([0 1]);
cb = colorbar;
cb.Ticks = [0, 1];
cb.TickLabels = {'Invisible', 'Visible'};
cb_pos = cb.Position;
x_text = cb_pos(1) + cb_pos(3) + 0.01;
y_invisible = 0.25;
y_visible = 0.75;
annotation('textbox', [x_text, cb_pos(2) + cb_pos(4)*y_invisible, 0.06, 0.03], ...
           'String', 'Invisible', 'EdgeColor', 'none', 'Color', [0 0.6 0], ...
           'FontWeight', 'bold', 'HorizontalAlignment', 'left');
annotation('textbox', [x_text, cb_pos(2) + cb_pos(4)*y_visible, 0.06, 0.03], ...
           'String', 'Visible', 'EdgeColor', 'none', 'Color', [1 0 0], ...
           'FontWeight', 'bold', 'HorizontalAlignment', 'left');
legend([s, h_radar, h_path, h_start, h_end],{'Visibility Map', 'Radar Position', ...
        'Optimized Path','Start Position', 'End Position'},'Location', 'best');
view(-20, 85);
title('LOS Visibility of RADAR');
xlabel('X [km]');
ylabel('Y [km]');
zlabel('Altitude (meters)');
grid on;

%% 가시성까지 고려된 환경에서 PSO 테스트

% 이전 평지에서 레이더 위치
% radar_1 = [10000, 10000, 230]; % 단일 레이더의 경우
% radars = [10000, 10000, 230]; % 복수의 레이더 경우
% 산지에서 레이더 위치
% radar_1 = [45000, 60000, 750]; % 단일 레이더의 경우
% radars = [45000, 60000, 750]; % 복수의 레이더 경우
% simple terrain에서 단일 레이더(가시성 항 검증)
% radars = double([6600, 7100, 12]);
% start_pos = [6900, 21000, 30];
% end_pos = [25000, 5000, 30];
% 이전 평지에서 시작점 및 종료점
% start_pos = [0, 0, 200];
% end_pos = [25000,34000,80];
% 산지에서 시작점 및 종료점
% start_pos = [11000, 16000, 150];
% end_pos = [60000, 60000, 100];
% end_pos = [1780, 5180, 450];
% very simple terrain에서 복수의 레이더 설치 위치
radars = [
    21000, 11000, 12;
    25000, 22000, 12;
    18000, 26000, 12;
    11000, 5000, 12; 
    12000, 18000, 12];
start_pos = [28000, 12000, 30];
end_pos = [3500, 17000, 30];
[path, sir_values, visibility_values] = PSO_visibility(radars, start_pos, end_pos, ...
    X, Y, Z, RADAR,visibility_matrix);

%% PSO 결과 시각화

% visualize_PSO_gray(path, SIR_matrix, radar_1, X, Y, Z, start_pos, end_pos);
% 색상 맞춰야하나?
visualize_PSO_SIR(path, SIR_matrix, radars, X, Y, Z, start_pos, end_pos);

%% path와 sir_data 길이 맞춤

% num_waypoints = size(path, 1);
% current_sir_data_length = length(sir_data);
% if current_sir_data_length < num_waypoints
%     last_sir_matrix = sir_data{end}; % sir_data의 마지막 데이터 복사
%     for k = (current_sir_data_length+1):num_waypoints
%         sir_data{k} = last_sir_matrix; % 부족한 부분에 동일 데이터 채우기
%     end
% end

%% 지형 불러오기

radar_1 = [10000, 10000, 230];
% load C:/Users/leeyj/lab_ws/data/VTD/RADAR/map_flat_land.mat;
load C:/Users/leeyj/lab_ws/data/VTD/RADAR/DTED_mountain.mat;
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
% visualize_PSO_SIR_2(path, sir_data, sir_values, visibility_values, radar_1, X, Y, Z);
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

%% 가시성까지 고려된 환경에서 PSO 테스트

[path, sir_values] = PSO_visibility(radars, start_pos, end_pos, X, Y, Z, RADAR,visibility_matrix);

%% PSO 결과 시각화

% visualize_PSO_gray(path, sir_data, radar_1, X, Y, Z);
% visulaize_PSO_SIR_3(path, SIR_matrix, );

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