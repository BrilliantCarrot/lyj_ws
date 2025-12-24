%% Psuedo 6 DoF Simulation
% simulation for heliocopter case
clc; 
clear; 
close all;
% 시뮬레이션 조건 정의
r_m0 = [0; 0; 0];
v_m0 = [200; 0; 0];
r_t0 = [5000; 1100; 1000];
v_t0 = [0; 0; 0];
N = 4;
% 헬기 및 표적의 위치, 속도
r_m = r_m0;
v_m = v_m0;
r_t = r_t0;
v_t = v_t0;
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
attitude = [0; 0; 0]; % [roll; pitch; yaw] in radians

% 동역학 오소(추력,항력,중력 고려)
mass = 7000; % 헬기 중량 (kg)
thrust = 10000; % 추력 (N)
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
time_record = [];

%% 시뮬레이션
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
    yaw = attitude(3); % yaw는 변하지 않는다고 가정
    attitude = [roll; pitch; yaw]; % 새로운 자세 저장
    % 회전 행렬 적용 (3-2-1 변환 적용)
    % 기체 고정 좌표계를 지면좌표계에 대해 표시
    R = eul2rotm([yaw, pitch, roll], 'ZYX'); % 회전 행렬 생성
    total_acceleration_2 = R * total_acceleration; % 회전 변환 적용

    % 오일러 적분 적용
    xm(1:3,1) = xm(1:3,1) + xm(4:6,1) * dt; % 위치 업데이트
    % xm(4:6,1) = xm(4:6,1) + u_ppn * dt; % 속도 업데이트
    xm(4:6,1) = xm(4:6,1) + total_acceleration_2 * dt; % 속도 업데이트    
    xm_record1 = [xm_record1; xm.'];
    xt(1:3,1) = xt(1:3,1) + xt(4:6,1) * dt; % 위치 업데이트
    xt(4:6,1) = xt(4:6,1) + u_t * dt; % 속도 업데이트
    xt_record1 = [xt_record1; xt.'];
    % Miss Distance 업데이트
    current_miss_distance = norm(r_tm);
    miss_distance_record = [miss_distance_record; current_miss_distance];
    if current_miss_distance < miss_distance
        miss_distance = current_miss_distance;
    end

    % 값 저장
    los_rate_record = [los_rate_record; OMEGA_tm'*(180/pi)];
    zem_record = [zem_record; zem];
    time_record = [time_record; t];
    attitude_record = [attitude_record; attitude.'];
    t = t + dt;
end
xm_record1 = xm_record1(1:end-1, :); % 마지막 행 제거, 시간 스탭 맞추기 용 (30001 → 30000)
time_vector = 0:dt:30-dt; % 0~30초 범위 시간 벡터
time_vector_2 = [time_vector, time_vector(end) + dt]; % 마지막 시간 추가 (30000 → 30001)
%% 종료조건이 t>30인 경우 시각화
close all;
figure;
plot3(xm_record1(:,1),xm_record1(:,2),xm_record1(:,3),'b','LineWidth',2)
hold on
plot3(r_m0(1,1),r_m0(2,1),r_m0(3,1),'bp','LineWidth',2)
% plot3(xt_record1(:,1),xt_record1(:,2),xt_record1(:,3),'r','LineWidth',2)
plot3(r_t0(1,1),r_t0(2,1),r_t0(3,1),'rp','LineWidth',2)
legend({'헬기 궤적','헬기 초기 위치','타겟 초기 위치'},'Location','northwest','NumColumns',1)
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
grid on
title("PPN 적용 시뮬레이션 결과 (오일러 적분)")
hold off
% 헬기의 유도 과정 시각화
figure;
hold on
plot3(r_t0(1), r_t0(2), r_t0(3), 'rp', 'MarkerSize', 10, 'LineWidth', 2); % 타겟 초기 위치
plot3(xm_record1(:,1), xm_record1(:,2), xm_record1(:,3), 'b--'); % 전체 궤적 (점선)
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
grid on;
title('헬기 유도 궤적 애니메이션');
view(3);

for i = 1:20:length(time_record) % 애니메이션 속도 조절
    plot3(xm_record1(i,1), xm_record1(i,2), xm_record1(i,3), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
    pause(0.01);
end
hold off;
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

%% 종료조건이 zem<0.0001인 경우 시각화
close all;
figure;
plot3(xm_record1(:,1),xm_record1(:,2),xm_record1(:,3),'b','LineWidth',2)
hold on
plot3(r_m0(1,1),r_m0(2,1),r_m0(3,1),'bp','LineWidth',2)
% plot3(xt_record1(:,1),xt_record1(:,2),xt_record1(:,3),'r','LineWidth',2)
plot3(r_t0(1,1),r_t0(2,1),r_t0(3,1),'rp','LineWidth',2)
legend({'헬기 궤적','헬기 초기 위치','타겟 초기 위치'},'Location','northwest','NumColumns',1)
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


%% ode 45를 통한 시뮬레이션 수행
close all;

[t_m, xm_record] = ode45(@(t, xm) missile_dynamics(t, xm, xt0, N), tspan, xm0);
[t_t, xt_record] = ode45(@(t, xt) target_dynamics(t, xt, xm0, N), tspan, xt0);

figure;
plot3(xm_record(:,1), xm_record(:,2), xm_record(:,3), 'black', 'LineWidth', 2);
hold on;
% plot3(xt_record(:,1), xt_record(:,2), xt_record(:,3), 'r', 'LineWidth', 2);
plot3(r_m0(1), r_m0(2), r_m0(3), 'bp', 'MarkerSize', 10, 'LineWidth', 2);
plot3(r_t0(1), r_t0(2), r_t0(3), 'rp', 'MarkerSize', 10, 'LineWidth', 2);
legend({'미사일 궤적', '미사일 초기 위치', '타겟 초기 위치'}, 'Location', 'northwest');
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
grid on;
title("PPN 적용 시뮬레이션 결과");
hold off;
figure;
subplot(4,1,1);
plot(t_m, xm_record(:,4), 'b', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('x 방향 속도 [m/s]');
title('미사일 x 방향 속도 변화');
grid on;
subplot(4,1,2);
plot(t_m, xm_record(:,5), 'r', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('y 방향 속도 [m/s]');
title('미사일 y 방향 속도 변화');
grid on;
subplot(4,1,3);
plot(t_m, xm_record(:,6), 'g', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('z 방향 속도 [m/s]');
title('미사일 z 방향 속도 변화');
grid on;
subplot(4,1,4);
speed_norm = vecnorm(xm_record(:,4:6), 2, 2); % 속도 벡터의 norm 계산
plot(t_m, speed_norm, 'k', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('vector norm [m/s]');
title('미사일 속력 변화');
grid on;

%% PPN 시뮬레이션 수행 - RK4 함수 이용

close all;
while norm(xm(1:3,1) - xt(1:3,1)) > 1
    % [유도 명령에 따른 유도탄의 수치적분]
    Num = 1;
    while Num < 5
        deriv = [xm(4:6,1); u_ppn];
        [xm, t, save_state_m, sum_k_m, Num] = RK4(xm, deriv, save_state_m, sum_k_m, t, Num, dt);
    end    
    xm_record1 = [xm_record1; xm.'];

    t = t_step;

    % [기동에 따른 표적의 수치적분]
    Num = 1;
    while Num < 5
        deriv = [xt(4:6,1); u_t];
        [xt, t, save_state_t, sum_k_t, Num] = RK4(xt, deriv, save_state_t, sum_k_t, t, Num, dt);
    end    
    xt_record1 = [xt_record1; xt.'];

    % [업데이트 된 상태로 새로운 명령 생성]
    % 유도탄 위치, 속도 정의
    r_m = xm(1:3,1);
    v_m = xm(4:6,1);
    
    % 표적 위치, 속도 정의
    r_t = xt(1:3,1);
    v_t = xt(4:6,1);
    
    % 잔여거리 벡터 및 상대 속도 계산
    r_tm = r_t - r_m;
    v_tm = v_t - v_m;
    v_mt = v_m - v_t;
    
    % 3차원 공간에서 LOS rate 계산
    OMEGA_tm = cross(r_tm, v_tm) / (norm(r_tm))^2;
    
    % 3차원 PPN 유도 명령 생성
    u_ppn = N * cross(OMEGA_tm, v_m);
    
    % 도망가는 표적에 대한 조건 - LOS rate가 커지게 기동
    u_t = - N * cross(OMEGA_tm, v_t);
    
    t_step = t;
end
duration_ppn = t;

plot3(xm_record1(:,1),xm_record1(:,2),xm_record1(:,3),'b','LineWidth',2)
hold on
plot3(xt_record1(:,1),xt_record1(:,2),xt_record1(:,3),'r','LineWidth',2)
plot3(r_m0(1,1),r_m0(2,1),r_m0(3,1),'bp','LineWidth',2)
plot3(r_t0(1,1),r_t0(2,1),r_t0(3,1),'rp','LineWidth',2)
legend({'미사일 궤적','타겟 궤적','미사일 초기 위치','타겟 초기 위치'},'Location','northwest','NumColumns',1)
hold on
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
grid on
title("PPN 적용 시뮬레이션 결과")
hold off



%% PPN 시간 출력

fprintf(['ppn 적용 시 소요시간 : ', num2str(duration_ppn), ' sec'])

%% PPN 유도탄 속도 출력

for i = 1:size(xm_record1,1)
    vm_ppn(1,i) = norm(xm_record1(i,4:6));
end
t_list1 = linspace(0,duration_ppn,size(xm_record1,1));
plot(t_list1, vm_ppn,'b','LineWidth',2)

grid on 
xlabel('t [sec]')
ylabel('m/s^2')
xlim([0 duration_ppn])
title("PPN 적용 시 시간에 따른 유도탄 속력")
hold off

%% ode45 내부 함수

function dxm = missile_dynamics(~, xm, xt, N)
    % 미사일 위치 및 속도 분리
    r_m = xm(1:3);
    v_m = xm(4:6);
    % 표적 위치 및 속도 분리
    r_t = xt(1:3);
    v_t = xt(4:6);
    % LOS 벡터 및 상대 속도 계산
    r_tm = r_t - r_m;
    v_tm = v_t - v_m;
    % LOS 회전 속도
    OMEGA_tm = cross(r_tm, v_tm) / (norm(r_tm))^2;
    % PPN 유도 명령 생성
    u_ppn = N * cross(OMEGA_tm, v_m);
    % 미사일 운동 방정식
    dxm = [v_m; u_ppn];
end

function dxt = target_dynamics(~, xt, ~, ~)
    % 표적 속도는 0 (고정된 표적)
    dxt = [xt(4:6); 0; 0; 0];
end