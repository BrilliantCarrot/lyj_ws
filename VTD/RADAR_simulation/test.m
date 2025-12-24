% Psuedo 6 DoF Simulation
% simulation for heliocopter case
clc; clear; close all;
% 레이더 파라미터
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
% 시뮬레이션 조건 정의
r_m0 = [0; 0; 0];
v_m0 = [200; 0; 0];
% r_t0 = [5000; 1100; 1000];
r_t0 = [5000; 1100; 1000];
v_t0 = [0; 0; 0];
N = 4;
% 헬기 및 표적의 위치, 속도
r_m = r_m0;
v_m = v_m0;
r_t = r_t0;
v_t = v_t0;
% 레이더 요소 설정
radar_pos = [2500; 500; 20];  % 레이더 위치 [x; y; z]
az_record = [];  % 방위각 저장
el_record = [];  % 고각 저장
% 잔여거리 벡터 및 상대 속도 계산
r_tm = r_t - r_m;
v_tm = v_t - v_m;
v_mt = v_m - v_t;
% 3차원 공간에서 LOS rate 계산
OMEGA_tm = cross(r_tm, v_tm) / (norm(r_tm))^2;
% 3차원 PPN 유도 명령 생성
u_ppn = N * cross(OMEGA_tm, v_m);
% 타겟의 속도 0 m/s(고정된 타겟)
u_t = [0; 0; 0];
zem = inf;

t = 0;
dt = 0.001;

% 초기 자세 (Euler angles: roll, pitch, yaw)
attitude = [0; 0; 0]; % [roll; pitch; yaw] [radian]

% 동역학 요소(추력,항력,중력 고려)
mass = 7000; % 헬기 중량 (kg)
thrust = 30000; % 추력 (N)
Cd = 0.1; % 항력 계수
rho = 1.225; % 공기 밀도 (kg/m^3)
A = 1; % 단면적 (m^2)
g = 9.81; % 중력 가속도 (m/s^2)

% 결과 저장 변수 정의
xm0 = [r_m0; v_m0]; % 초기 위치
xt0 = [r_t0; v_t0]; % 초기 위치
xm = [r_m0; v_m0];
xm_record1 = xm.';
xt = [r_t0; v_t0];
xt_record1 = xt.';
attitude_record = attitude.'; % 자세 저장
required_thrust_record = []; % 필요 추력 저장
los_rate_record = [];
OMEGA_tm_record = [];
zem_record = [];
miss_distance = inf; % 초기 Miss Distance 설정(무한대로 설정)
miss_distance_record = [];
rcs_record = [];
time_record = [];
%% 헬기 기동 시뮬레이션
% while t < 30
while zem > 0.0001
    % 헬기 및 표적의 위치, 속도
    r_m = xm(1:3,1);
    v_m = xm(4:6,1);
    r_t = xt(1:3,1);
    v_t = xt(4:6,1);
    % 잔여거리 벡터 및 상대 속도 계산
    r_tm = r_t - r_m;
    v_tm = v_t - v_m;
    v_mt = v_m - v_t;
    % 3차원 공간에서 LOS rate 계산
    OMEGA_tm = cross(r_tm, v_tm) / (norm(r_tm))^2;
    % OMEGA_tm_record = [OMEGA_tm_record; OMEGA_tm']; % LOS rate 기록
    los_rate = norm(OMEGA_tm); % LOS rate 크기 저장
    % ZEM 계산
    t_go = norm(r_tm) / norm(v_tm); % 예상 충돌 시간
    zem = norm(r_tm + v_tm * t_go); % ZEM 계산
    % 3차원 PPN 유도 명령 생성
    u_ppn = N * cross(OMEGA_tm, v_m);
    % 속도 크기 계산
    v_magnitude = norm(v_m);
    % 항력 계산 (Drag = 1/2 * Cd * rho * v^2 * A)
    drag = 0.5 * Cd * rho * v_magnitude^2 * A * pi;
    drag_force = -drag * (v_m / v_magnitude); % 속도 방향 반대
    % 중력 적용
    gravity_force = [0; 0; -mass * g];
    % 총 가속도 (추력 포함)
    % 헬기의 총 가속도는 유도 명령에서 항력과 중력을 뺀 값
    % 원하는 가속도를 가지기 위해 필요 추력을 역산해야 함
    % 필요한 추력 = 유도 가속도 - 항력 - 중력
    % 필요한 추력을 이용하여 가속도 업데이트
    required_thrust = mass * (u_ppn - drag_force / mass - gravity_force / mass);
    total_acceleration = (required_thrust + drag_force + gravity_force) / mass;
    required_thrust_record = [required_thrust_record; required_thrust.'];

    % 헬기 자세(roll, pitch) 계산
    thrust_direction = required_thrust / norm(required_thrust); % 단위 벡터
    roll = atan2(thrust_direction(2), thrust_direction(3)); % y축과 z축을 이용한 roll 계산
    pitch = atan2(-thrust_direction(1), sqrt(thrust_direction(2)^2 + thrust_direction(3)^2)); % x축 방향 고려
    vx = v_m(1); vy = v_m(2);
    yaw = atan2(vy,vx);
    % yaw = atan2(thrust_direction(2), thrust_direction(1));

    attitude = [roll; pitch; yaw]; % 자세 저장
    % 회전 행렬 적용 (3-2-1 변환 적용)
    % 기체 고정 좌표계를 지면좌표계에 대해 표시
    R = eul2rotm([yaw, pitch, roll], 'ZYX'); % 회전 행렬 생성
    total_acceleration_2 = R * total_acceleration; % 회전 행렬 적용

    % 오일러 적분 적용
    xm(1:3,1) = xm(1:3,1) + xm(4:6,1) * dt; % 비행체 위치 업데이트
    xm(4:6,1) = xm(4:6,1) + u_ppn * dt; % 비행체 속도 업데이트
    % xm(4:6,1) = xm(4:6,1) + total_acceleration_2 * dt; % 속도 업데이트    
    xm_record1 = [xm_record1; xm.'];
    xt(1:3,1) = xt(1:3,1) + xt(4:6,1) * dt; % 타겟 위치 업데이트
    xt(4:6,1) = xt(4:6,1) + u_t * dt; % 타겟 속도 업데이트
    xt_record1 = [xt_record1; xt.'];
    % Miss Distance 업데이트
    current_miss_distance = norm(r_tm);
    miss_distance_record = [miss_distance_record; current_miss_distance];
    if current_miss_distance < miss_distance
        miss_distance = current_miss_distance;
    end

    % 레이더 관측 기하 계산
    % 관측 기하는 RADAR_Module 내부 수식 이용
    r_rel = r_m - radar_pos; % 레이더 → 헬기의 LOS 벡터 계산
    % az = atan2(r_rel(2), r_rel(1)) * (180/pi);
    % el_original = atan2(r_rel(3), sqrt(r_rel(1)^2 + r_rel(2)^2)) * (180/pi);
    % el = 90 - el_original;
    % los_pitch = atan2(-r_rel(3), norm(r_rel(1:2)));
    los_pitch = atan2( r_rel(3), sqrt( r_rel(1)^2 + r_rel(2)^2 ) );
    los_yaw = atan2(r_rel(2), r_rel(1));    
    pitch = Angle_trim(los_pitch);
    yaw = Angle_trim(los_yaw);
    el = pitch;
    az = yaw;
    el_deg = el * (180/pi);
    az_deg = az * (180/pi);
    pitch_array = RADAR.theta(1, :) * pi/180;
    yaw_array = RADAR.psi(:, 1) * pi/180;
    p_idx = Find_Index(pitch_array, length(pitch_array), pitch);
    y_idx = Find_Index(yaw_array, length(yaw_array), yaw);
    p_lower = rcs_table(:, p_idx);
    p_upper = rcs_table(:, p_idx + 1);
    p_rcs = p_lower + (pitch - pitch_array(p_idx)) * (p_upper - p_lower) / (pitch_array(p_idx + 1) - pitch_array(p_idx));
    y_lower = p_rcs(y_idx, :);
    y_upper = p_rcs(y_idx + 1, :);
    rcs = y_lower + (yaw - yaw_array(y_idx)) * (y_upper - y_lower) / (yaw_array(y_idx + 1) - yaw_array(y_idx));
    % rcs = 10^(rcs / 10);  % dB to linear scale
    
    % 값 저장
    los_rate_record = [los_rate_record; OMEGA_tm'*(180/pi)]; % Los Rate 저장
    zem_record = [zem_record; zem]; % ZEM 저장
    time_record = [time_record; t]; % 시간 저장
    attitude_record = [attitude_record; attitude.']; % 자세(롤,피치,요) 저장
    az_record = [az_record; az_deg]; % 방위각 저장
    el_record = [el_record; el_deg]; % 고각 저장
    rcs_record = [rcs_record; rcs]; % rcs 저장
    t = t + dt;
end
xm_record1 = xm_record1(1:end-1, :); % 마지막 행 제거, 시간 스탭 맞추기 용 (30001 → 30000)
time_vector = 0:dt:30-dt; % 0~30초 범위 시간 벡터
time_vector_2 = [time_vector, time_vector(end) + dt]; % 마지막 시간 추가 (30000 → 30001)
attitude_record = attitude_record(2:end, :);
%% 유도탄 시뮬레이션
% % while t < 30
% while zem > 0.0001
%     % 유도탄 및 표적의 위치, 속도
%     r_m = xm(1:3,1);
%     v_m = xm(4:6,1);
%     r_t = xt(1:3,1);
%     v_t = xt(4:6,1);
%     % 잔여거리 벡터 및 상대 속도 계산
%     r_tm = r_t - r_m;
%     v_tm = v_t - v_m;
%     v_mt = v_m - v_t;
%     % 3차원 공간에서 LOS rate 계산
%     OMEGA_tm = cross(r_tm, v_tm) / (norm(r_tm))^2;
%     % OMEGA_tm_record = [OMEGA_tm_record; OMEGA_tm']; % LOS rate 기록
%     los_rate = norm(OMEGA_tm); % LOS rate 크기 저장
%     % ZEM 계산
%     t_go = norm(r_tm) / norm(v_tm); % 예상 충돌 시간
%     zem = norm(r_tm + v_tm * t_go); % ZEM 계산
%     % 3차원 PPN 유도 명령 생성
%     u_ppn = N * cross(OMEGA_tm, v_m);
% 
%     % 속도 크기 계산
%     v_magnitude = norm(v_m);
%     % 항력 계산 (Drag = 1/2 * Cd * rho * v^2 * A)
%     drag = 0.5 * Cd * rho * v_magnitude^2 * A * pi;
%     drag_force = -drag * (v_m / v_magnitude); % 속도 방향 반대
%     % 중력 적용
%     gravity_force = [0; 0; -mass * g];
%     required_thrust = thrust * (v_m / v_magnitude); % 속도 방향으로 추력 고정
%     guidance_force = mass * u_ppn;
%     total_force = required_thrust + drag_force + gravity_force + guidance_force;
%     total_acceleration = total_force / mass;
%     required_thrust_record = [required_thrust_record; required_thrust.'];
% 
%     % 헬기 자세(roll, pitch) 계산
%     thrust_direction = total_acceleration / norm(total_acceleration); % 단위 벡터
%     roll = atan2(thrust_direction(2), thrust_direction(3)); % y축과 z축을 이용한 roll 계산
%     pitch = atan2(-thrust_direction(1), sqrt(thrust_direction(2)^2 + thrust_direction(3)^2)); % x축 방향 고려
%     yaw = attitude(3); % yaw는 변하지 않는다고 가정
%     attitude = [roll; pitch; yaw]; % 새로운 자세 저장
%     % 회전 행렬 적용 (3-2-1 변환 적용)
%     % 기체 고정 좌표계를 지면좌표계에 대해 표시
%     R = eul2rotm([yaw, pitch, roll], 'ZYX'); % 회전 행렬 생성
%     total_acceleration_2 = R * total_acceleration; % 회전 변환 적용
% 
%     % 오일러 적분 적용
%     xm(1:3,1) = xm(1:3,1) + xm(4:6,1) * dt; % 위치 업데이트
%     % xm(4:6,1) = xm(4:6,1) + u_ppn * dt; % 속도 업데이트
%     xm(4:6,1) = xm(4:6,1) + total_acceleration_2 * dt; % 속도 업데이트    
%     xm_record1 = [xm_record1; xm.'];
%     xt(1:3,1) = xt(1:3,1) + xt(4:6,1) * dt; % 위치 업데이트
%     xt(4:6,1) = xt(4:6,1) + u_t * dt; % 속도 업데이트
%     xt_record1 = [xt_record1; xt.'];
%     % Miss Distance 업데이트
%     current_miss_distance = norm(r_tm);
%     miss_distance_record = [miss_distance_record; current_miss_distance];
%     if current_miss_distance < miss_distance
%         miss_distance = current_miss_distance;
%     end
% 
%     % 값 저장
%     los_rate_record = [los_rate_record; OMEGA_tm'*(180/pi)];
%     zem_record = [zem_record; zem];
%     time_record = [time_record; t];
%     attitude_record = [attitude_record; attitude.'];
%     t = t + dt;
% end
% xm_record1 = xm_record1(1:end-1, :); % 마지막 행 제거, 시간 스탭 맞추기 용 (30001 → 30000)
% time_vector = 0:dt:30-dt; % 0~30초 범위 시간 벡터
% time_vector_2 = [time_vector, time_vector(end) + dt]; % 마지막 시간 추가 (30000 → 30001)
%% 종료조건이 t>30인 경우 시각화
% close all;
figure;
plot3(xm_record1(:,1),xm_record1(:,2),xm_record1(:,3),'b','LineWidth',2)
hold on
plot3(r_m0(1,1),r_m0(2,1),r_m0(3,1),'bp','LineWidth',2)
% plot3(xt_record1(:,1),xt_record1(:,2),xt_record1(:,3),'r','LineWidth',2)
plot3(r_t0(1,1),r_t0(2,1),r_t0(3,1),'rp','LineWidth',2)
plot3(radar_pos(1), radar_pos(2), radar_pos(3), 'kd', 'LineWidth', 2)
legend({'헬기 궤적','헬기 초기 위치','타겟 초기 위치','레이더 위치'},'Location','northwest','NumColumns',1)
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
grid on
title("PPN 적용 시뮬레이션 결과 (오일러 적분)")
hold off
% 헬기의 유도 과정 시각화
% figure;
% hold on
% plot3(r_t0(1), r_t0(2), r_t0(3), 'rp', 'MarkerSize', 10, 'LineWidth', 2); % 타겟 초기 위치
% plot3(xm_record1(:,1), xm_record1(:,2), xm_record1(:,3), 'b--'); % 전체 궤적 (점선)
% xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
% grid on;
% title('헬기 유도 궤적 애니메이션');
% view(3);
% 
% for i = 1:20:length(time_record) % 애니메이션 속도 조절
%     plot3(xm_record1(i,1), xm_record1(i,2), xm_record1(i,3), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
%     pause(0.01);
% end
% hold off;
% Los rate 
figure;
subplot(3,1,1);
plot(time_vector,los_rate_record(:,1), 'b', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('LOS rate x축 [rad/s]');
title('LOS rate 변화 (x 방향)');
grid on;
subplot(3,1,2);
plot(time_vector,los_rate_record(:,2), 'r', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('LOS rate y축 [rad/s]');
title('LOS rate 변화 (y 방향)');
grid on;
subplot(3,1,3);
plot(time_vector,los_rate_record(:,3), 'g', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('LOS rate z축 [rad/s]');
title('LOS rate 변화 (z 방향)');
grid on;
% Miss Distance 및 ZEM
figure;
% subplot(2,1,1);
% plot(time_vector, los_rate_record, 'k', 'LineWidth', 1.5)
% xlabel('Time step'), ylabel('LOS rate [rad/s]')
% grid on, title('LOS rate 변화')
subplot(2,1,1);
plot(time_vector, miss_distance_record, 'b', 'LineWidth', 1.5)
xlabel('Time step'), ylabel('Miss Distance [m]')
grid on, title('Miss Distance 변화')
subplot(2,1,2);
plot(time_vector, zem_record, 'm', 'LineWidth', 1.5)
xlabel('Time step'), ylabel('ZEM [m]')
grid on, title('Zero Effort Miss (ZEM)')
% 위치
figure;
subplot(3,1,1);
plot(time_vector, xm_record1(:,1), 'b', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('x 방향 위치 [m]');
title('x 방향 위치 변화');
grid on;
subplot(3,1,2);
plot(time_vector, xm_record1(:,2), 'r', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('y 방향 위치 [m]');
title('y 방향 위치 변화');
grid on;
subplot(3,1,3);
plot(time_vector, xm_record1(:,3), 'g', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('z 방향 위치 [m]');
title('z 방향 위치 변화');
grid on;
% 속도
figure;
subplot(3,1,1);
plot(time_vector, xm_record1(:,4), 'b', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('x 방향 속도 [m/s]');
title('x 방향 속도 변화');
grid on;
subplot(3,1,2);
plot(time_vector, xm_record1(:,5), 'r', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('y 방향 속도 [m/s]');
title('y 방향 속도 변화');
grid on;
subplot(3,1,3);
plot(time_vector, xm_record1(:,6), 'g', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('z 방향 속도 [m/s]');
title('z 방향 속도 변화');
% 속력
figure;
speed_norm = vecnorm(xm_record1(:,4:6), 2, 2); % 속도 벡터의 norm 계산
plot(time_vector, speed_norm, 'b', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('Speed [m/s]');
title('속력 변화');
grid on;
grid on;
% 가속도
acc_x = diff(xm_record1(:,4)) / dt;
acc_y = diff(xm_record1(:,5)) / dt;
acc_z = diff(xm_record1(:,6)) / dt;
time_vector_trimmed = time_vector(1:end-1);
figure;
subplot(3,1,1);
plot(time_vector_trimmed, acc_x, 'b', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('x 가속도 [m/s²]');
title('x 방향 가속도 변화');
grid on;
subplot(3,1,2);
plot(time_vector_trimmed, acc_y, 'r', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('y 가속도 [m/s²]');
title('y 방향 가속도 변화');
grid on;
subplot(3,1,3);
plot(time_vector_trimmed,acc_z, 'g', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('z 가속도 [m/s²]');
title('z 방향 가속도 변화');
grid on;
% 롤, 피치, 요
attitude_deg = attitude_record * (180/pi);
figure
subplot(3,1,1);
plot(time_record, attitude_deg(:,1), 'b', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('roll [deg]');
title('Roll (φ)');
grid on;
attitude_deg = attitude_record * (180/pi);
subplot(3,1,2);
plot(time_record, attitude_deg(:,2), 'b', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('pitch [deg]');
title('Pitch (θ)');
grid on;
attitude_deg = attitude_record * (180/pi);
subplot(3,1,3);
plot(time_record, attitude_deg(:,3), 'b', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('yaw [deg]');
title('Yaw (ψ)');
grid on;
% 고각 및 방위각
figure();
subplot(2,1,1)
plot(time_record, az_record, 'b','LineWidth',2); grid on;
xlabel('시간 [s]'); ylabel('Azimuth [deg]');
title('관측기하에 따른 방위각 (φ)');
subplot(2,1,2)
plot(time_record, el_record, 'r','LineWidth',2); grid on;
xlabel('시간 [s]'); ylabel('Elevation [deg]');
title('관측기하에 따른 고각 (θ)');
% rcs
figure();
plot(time_record, rcs_record, 'b','LineWidth',2); grid on;
xlabel('시간 [s]'); ylabel('RCS [dB]');
title('비행체 RCS');
%% 종료조건이 zem<0.0001인 경우 시각화
% close all;
figure;
plot3(xm_record1(:,1),xm_record1(:,2),xm_record1(:,3),'b','LineWidth',2)
hold on
plot3(r_m0(1,1),r_m0(2,1),r_m0(3,1),'bp','LineWidth',2)
% plot3(xt_record1(:,1),xt_record1(:,2),xt_record1(:,3),'r','LineWidth',2)
plot3(r_t0(1,1),r_t0(2,1),r_t0(3,1),'rp','LineWidth',2)
plot3(radar_pos(1), radar_pos(2), radar_pos(3), 'kd', 'LineWidth', 2)
legend({'헬기 궤적','헬기 초기 위치','타겟 초기 위치','레이더 위치'},'Location','northwest','NumColumns',1)
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
grid on
title("PPN 적용 시뮬레이션 결과 (오일러 적분)")
hold off
% Los rate 
figure;
subplot(3,1,1);
plot(time_record,los_rate_record(:,1), 'b', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('LOS rate x축 [rad/s]');
title('LOS rate 변화 (x 방향)');
grid on;
subplot(3,1,2);
plot(time_record,los_rate_record(:,2), 'r', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('LOS rate y축 [rad/s]');
title('LOS rate 변화 (y 방향)');
grid on;
subplot(3,1,3);
plot(time_record,los_rate_record(:,3), 'g', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('LOS rate z축 [rad/s]');
title('LOS rate 변화 (z 방향)');
grid on;
% Miss Distance 및 ZEM
figure;
% subplot(2,1,1);
% plot(time_vector, los_rate_record, 'k', 'LineWidth', 1.5)
% xlabel('Time step'), ylabel('LOS rate [rad/s]')
% grid on, title('LOS rate 변화')
subplot(2,1,1);
plot(time_record, miss_distance_record, 'b', 'LineWidth', 1.5)
xlabel('Time step'), ylabel('Miss Distance [m]')
grid on, title('Miss Distance 변화')
subplot(2,1,2);
plot(time_record, zem_record, 'm', 'LineWidth', 1.5)
xlabel('Time step'), ylabel('ZEM [m]')
grid on, title('Zero Effort Miss (ZEM)')
% 위치
figure;
subplot(3,1,1);
plot(time_record, xm_record1(:,1), 'b', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('x 방향 위치 [m]');
title('x 방향 위치 변화');
grid on;
subplot(3,1,2);
plot(time_record, xm_record1(:,2), 'r', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('y 방향 위치 [m]');
title('y 방향 위치 변화');
grid on;
subplot(3,1,3);
plot(time_record, xm_record1(:,3), 'g', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('z 방향 위치 [m]');
title('z 방향 위치 변화');
grid on;
% 속도
figure;
subplot(3,1,1);
plot(time_record, xm_record1(:,4), 'b', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('x 방향 속도 [m/s]');
title('x 방향 속도 변화');
grid on;
subplot(3,1,2);
plot(time_record, xm_record1(:,5), 'r', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('y 방향 속도 [m/s]');
title('y 방향 속도 변화');
grid on;
subplot(3,1,3);
plot(time_record, xm_record1(:,6), 'g', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('z 방향 속도 [m/s]');
title('z 방향 속도 변화');
% 속력
figure;
speed_norm = vecnorm(xm_record1(:,4:6), 2, 2); % 속도 벡터의 norm 계산
plot(time_record, speed_norm, 'b', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('Speed [m/s]');
title('속력 변화');
grid on;
grid on;
% 가속도
acc_x = diff(xm_record1(:,4)) / dt;
acc_y = diff(xm_record1(:,5)) / dt;
acc_z = diff(xm_record1(:,6)) / dt;
time_vector_trimmed = time_record(1:end-1);
figure;
subplot(3,1,1);
plot(time_vector_trimmed, acc_x, 'b', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('x 가속도 [m/s²]');
title('x 방향 가속도 변화');
grid on;
subplot(3,1,2);
plot(time_vector_trimmed, acc_y, 'r', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('y 가속도 [m/s²]');
title('y 방향 가속도 변화');
grid on;
subplot(3,1,3);
plot(time_vector_trimmed,acc_z, 'g', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('z 가속도 [m/s²]');
title('z 방향 가속도 변화');
grid on;
% 롤, 피치, 요
attitude_deg = attitude_record * (180/pi);
figure
subplot(3,1,1);
plot(time_record, attitude_deg(:,1), 'b', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('roll [deg]');
title('Roll (φ)');
grid on;
attitude_deg = attitude_record * (180/pi);
subplot(3,1,2);
plot(time_record, attitude_deg(:,2), 'b', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('pitch [deg]');
title('Pitch (θ)');
grid on;
attitude_deg = attitude_record * (180/pi);
subplot(3,1,3);
plot(time_record, attitude_deg(:,3), 'b', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('yaw [deg]');
title('Yaw (ψ)');
grid on;
% 고각 및 방위각
figure();
subplot(2,1,1)
plot(time_record, az_record, 'b','LineWidth',2); grid on;
xlabel('시간 [s]'); ylabel('Azimuth [deg]');
title('관측기하에 따른 방위각 (φ)');
subplot(2,1,2)
plot(time_record, el_record, 'r','LineWidth',2); grid on;
xlabel('시간 [s]'); ylabel('Elevation [deg]');
title('관측기하에 따른 고각 (θ)');
% rcs
figure();
plot(time_record, rcs_record, 'b','LineWidth',2); grid on;
xlabel('시간 [s]'); ylabel('RCS [dB]');
title('비행체 RCS');
%% 기존 유도 알고리즘 결과 [롤,피치,요,추력]을 이용하여 궤적이 동일한지 확인
% chatgpt
% 고정 추력 적용하여 궤적 계산
% thrust = 30000;  % 고정된 추력값 [N]
% mass = 7000;     % 헬기 질량 [kg]
% dt = 0.001;
% g = 9.81;
% gravity_force = [0; 0; -mass * g];
% 
% % 초기 위치, 속도
% r_thrust = r_m0;
% v_thrust = v_m0;
% xm_record_thrust = r_thrust.';
% 
% for i = 1:size(attitude_record,1)
%     attitude = attitude_record(i, :); % roll, pitch, yaw
%     R = eul2rotm([attitude(3), attitude(2), attitude(1)], 'ZYX'); % Yaw-Pitch-Roll
%     thrust_vector = R * [0; 0; thrust]; % 고정 추력을 자세에 따라 회전
% 
%     % 가속도 = (추력 + 중력) / 질량
%     total_force = thrust_vector + gravity_force;
%     acc = total_force / mass;
% 
%     % 속도 및 위치 업데이트 (오일러 적분)
%     v_thrust = v_thrust + acc * dt;
%     r_thrust = r_thrust + v_thrust * dt;
% 
%     % 저장
%     xm_record_thrust = [xm_record_thrust; r_thrust.'];
% end
% 
% 
% % 궤적 비교 시각화
% figure;
% plot3(xm_record1(:,1),xm_record1(:,2),xm_record1(:,3),'b','LineWidth',2)
% hold on
% plot3(xm_record_thrust(:,1), xm_record_thrust(:,2), xm_record_thrust(:,3), 'g--', 'LineWidth', 2)
% plot3(r_m0(1,1),r_m0(2,1),r_m0(3,1),'bp','LineWidth',2)
% plot3(r_t0(1,1),r_t0(2,1),r_t0(3,1),'rp','LineWidth',2)
% plot3(radar_pos(1), radar_pos(2), radar_pos(3), 'kd', 'LineWidth', 2)
% legend({'PPN 유도 궤적','고정 추력 궤적','헬기 초기 위치','타겟 초기 위치','레이더 위치'},'Location','northwest')
% xlabel('x [m]')
% ylabel('y [m]')
% zlabel('z [m]')
% grid on
% title("PPN vs 고정 추력 기반 궤적 비교")
% hold off
%% 자세와 추력을 이용한 궤적 검증 시뮬레이션
% claude
% 원래 시뮬레이션에서 얻은 자세와 추력 기록을 활용하여 같은 궤적 생성

% 초기화
r_m_verify = r_m0;  % 초기 위치
v_m_verify = v_m0;  % 초기 속도
xm_verify = [r_m_verify; v_m_verify];
xm_record_verify = xm_verify.';

% 검증용 시뮬레이션 (이미 기록된 자세와 추력 사용)
for i = 1:length(time_record)
    % 현재 상태
    r_m_verify = xm_verify(1:3,1);
    v_m_verify = xm_verify(4:6,1);
    
    % i번째 기록된 자세와 추력 가져오기
    if i <= length(attitude_record)
        current_attitude = attitude_record(i, :)';
        roll = current_attitude(1);
        pitch = current_attitude(2);
        yaw = current_attitude(3);
    else
        % 배열 범위를 벗어나면 마지막 값 사용
        current_attitude = attitude_record(end, :)';
        roll = current_attitude(1);
        pitch = current_attitude(2);
        yaw = current_attitude(3);
    end
    
    if i <= length(required_thrust_record)
        current_thrust = required_thrust_record(i, :)';
    else
        % 배열 범위를 벗어나면 마지막 값 사용
        current_thrust = required_thrust_record(end, :)';
    end
    
    % 속도 크기 계산
    v_magnitude = norm(v_m_verify);
    
    % 항력 계산 (Drag = 1/2 * Cd * rho * v^2 * A)
    drag = 0.5 * Cd * rho * v_magnitude^2 * A * pi;
    drag_force = -drag * (v_m_verify / v_magnitude); % 속도 방향 반대
    
    % 중력 적용
    gravity_force = [0; 0; -mass * g];
    
    % 회전 행렬 생성 (3-2-1 변환)
    R = eul2rotm([yaw, pitch, roll], 'ZYX');
    
    % 기록된, 회전된 자세로 추력 적용 (지면 좌표계)
    total_acceleration = (current_thrust + drag_force + gravity_force) / mass;
    
    % 오일러 적분 적용
    xm_verify(1:3,1) = xm_verify(1:3,1) + xm_verify(4:6,1) * dt; % 위치 업데이트
    xm_verify(4:6,1) = xm_verify(4:6,1) + total_acceleration * dt; % 속도 업데이트
    
    % 결과 기록
    xm_record_verify = [xm_record_verify; xm_verify.'];
end

% 원래 궤적과 검증 궤적 비교
xm_record_verify = xm_record_verify(2:end, :); % 첫 행 제거하여 길이 맞추기

% 위치 오차 계산
position_error = xm_record1(:,1:3) - xm_record_verify(:,1:3);
position_error_norm = vecnorm(position_error, 2, 2);

% 속도 오차 계산
velocity_error = xm_record1(:,4:6) - xm_record_verify(:,4:6);
velocity_error_norm = vecnorm(velocity_error, 2, 2);

% 결과 시각화
close all;
figure;
plot3(xm_record1(:,1), xm_record1(:,2), xm_record1(:,3), 'b', 'LineWidth', 2);
hold on;
plot3(xm_record_verify(:,1), xm_record_verify(:,2), xm_record_verify(:,3), 'r--', 'LineWidth', 2);
plot3(r_m0(1,1), r_m0(2,1), r_m0(3,1), 'bp', 'LineWidth', 2);
plot3(r_t0(1,1), r_t0(2,1), r_t0(3,1), 'rp', 'LineWidth', 2);
legend({'원래 궤적', '검증 궤적', '초기 위치', '타겟 위치'}, 'Location', 'northwest', 'NumColumns', 1);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
grid on;
title('원래 궤적과 검증 궤적 비교');

% 위치 오차 시각화
figure;
subplot(4,1,1);
plot(time_record, position_error_norm, 'b', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('위치 오차 [m]');
title('위치 오차 크기');
grid on;

subplot(4,1,2);
plot(time_record, position_error(:,1), 'r', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('x 위치 오차 [m]');
title('x 방향 위치 오차');
grid on;

subplot(4,1,3);
plot(time_record, position_error(:,2), 'g', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('y 위치 오차 [m]');
title('y 방향 위치 오차');
grid on;

subplot(4,1,4);
plot(time_record, position_error(:,3), 'm', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('z 위치 오차 [m]');
title('z 방향 위치 오차');
grid on;

% 속도 오차 시각화
figure;
subplot(4,1,1);
plot(time_record, velocity_error_norm, 'b', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('속도 오차 [m/s]');
title('속도 오차 크기');
grid on;

subplot(4,1,2);
plot(time_record, velocity_error(:,1), 'r', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('x 속도 오차 [m/s]');
title('x 방향 속도 오차');
grid on;

subplot(4,1,3);
plot(time_record, velocity_error(:,2), 'g', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('y 속도 오차 [m/s]');
title('y 방향 속도 오차');
grid on;

subplot(4,1,4);
plot(time_record, velocity_error(:,3), 'm', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('z 속도 오차 [m/s]');
title('z 방향 속도 오차');
grid on;

% 최대 오차 출력
fprintf('최대 위치 오차: %.6f m\n', max(position_error_norm));
fprintf('최대 속도 오차: %.6f m/s\n', max(velocity_error_norm));
fprintf('평균 위치 오차: %.6f m\n', mean(position_error_norm));
fprintf('평균 속도 오차: %.6f m/s\n', mean(velocity_error_norm));
