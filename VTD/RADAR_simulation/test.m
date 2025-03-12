%% Psuedo 6 DoF Simulation
% simulation for heliocopter case
clc; 
clear; 
close all;
% 시뮬레이션 조건 정의
r_m0 = [0; 0; 0];
v_m0 = [200; 0; 0];
r_t0 = [2000; 1150; 300];
v_t0 = [0; 0; 0];
N = 4;
% 유도탄 및 표적의 위치, 속도
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

OMEGA_tm_record = [];  % LOS rate 기록용 배열

% 3차원 PPN 유도 명령 생성
u_ppn = N * cross(OMEGA_tm, v_m);
% 타겟의 속도 0 m/s(고정된 타겟)
u_t = [0; 0; 0];

t = 0;
dt = 0.001;
% 결과 저장 변수 정의
xm0 = [r_m0; v_m0];
xt0 = [r_t0; v_t0];
xm = [r_m0; v_m0];
xm_record1 = xm.';
xt = [r_t0; v_t0];
xt_record1 = xt.';

los_rate_record = []; % LOS rate 저장
zem_record = []; % ZEM 저장
miss_distance = inf; % 미스 거리 (무한대, 초기)
miss_distance_record = [];  % Miss Distance 저장 배열

% t_m = []; % 시간 기록용 배열

%% PPN 시뮬레이션 (오일러 적분 적용)
while t < 30
    % 유도탄 및 표적의 위치, 속도
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

    OMEGA_tm_record = [OMEGA_tm_record; OMEGA_tm']; % LOS rate 기록


    los_rate = norm(OMEGA_tm); % LOS rate 크기 저장
    % ZEM 계산
    t_go = norm(r_tm) / norm(v_tm); % 예상 충돌 시간
    zem = norm(r_tm + v_tm * t_go); % ZEM 계산
    % 3차원 PPN 유도 명령 생성
    u_ppn = N * cross(OMEGA_tm, v_m);
    % 오일러 적분 적용
    xm(1:3,1) = xm(1:3,1) + xm(4:6,1) * dt; % 위치 업데이트
    xm(4:6,1) = xm(4:6,1) + u_ppn * dt; % 속도 업데이트
    xm_record1 = [xm_record1; xm.'];
    xt(1:3,1) = xt(1:3,1) + xt(4:6,1) * dt; % 위치 업데이트
    xt(4:6,1) = xt(4:6,1) + u_t * dt; % 속도 업데이트
    xt_record1 = [xt_record1; xt.'];

    % 미스 거리 업데이트
    current_miss_distance = norm(r_tm);
    miss_distance_record = [miss_distance_record; current_miss_distance];
    if current_miss_distance < miss_distance
        miss_distance = current_miss_distance;
    end

    % 값 저장
    los_rate_record = [los_rate_record; los_rate];
    zem_record = [zem_record; zem];

    t = t + dt;
    % t_m = [t_m; t]; % 현재 시간 값 추가
end

duration_ppn = t;

%% PPN 시뮬레이션 결과 Plot
close all;
figure;
plot3(xm_record1(:,1),xm_record1(:,2),xm_record1(:,3),'b','LineWidth',2)
hold on
plot3(r_m0(1,1),r_m0(2,1),r_m0(3,1),'bp','LineWidth',2)
% plot3(xt_record1(:,1),xt_record1(:,2),xt_record1(:,3),'r','LineWidth',2)
plot3(r_t0(1,1),r_t0(2,1),r_t0(3,1),'rp','LineWidth',2)
legend({'미사일 궤적','미사일 초기 위치','타겟 초기 위치'},'Location','northwest','NumColumns',1)
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
grid on
title("PPN 적용 시뮬레이션 결과 (오일러 적분)")
hold off

figure;
subplot(3,1,1);
plot(OMEGA_tm_record(:,1), 'b', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('LOS rate x축 [rad/s]');
title('LOS rate 변화 (x 방향)');
grid on;
subplot(3,1,2);
plot(OMEGA_tm_record(:,2), 'r', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('LOS rate y축 [rad/s]');
title('LOS rate 변화 (y 방향)');
grid on;
subplot(3,1,3);
plot(OMEGA_tm_record(:,3), 'g', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('LOS rate z축 [rad/s]');
title('LOS rate 변화 (z 방향)');
grid on;


figure
subplot(3,1,1);
plot(los_rate_record, 'k', 'LineWidth', 1.5)
xlabel('Time step'), ylabel('LOS rate [rad/s]')
grid on, title('LOS rate 변화')
subplot(3,1,2);
plot(miss_distance_record, 'b', 'LineWidth', 1.5)
xlabel('Time step'), ylabel('Miss Distance [m]')
grid on, title('Miss Distance 변화')
subplot(3,1,3);
plot(zem_record, 'm', 'LineWidth', 1.5)
xlabel('Time step'), ylabel('ZEM [m]')
grid on, title('Zero Effort Miss (ZEM)')

figure;
subplot(3,1,1);
plot(xm_record1(:,1), 'b', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('x 방향 위치 [m]');
title('미사일 x 방향 위치 변화');
grid on;
subplot(3,1,2);
plot(xm_record1(:,2), 'r', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('y 방향 위치 [m]');
title('미사일 y 방향 위치 변화');
grid on;
subplot(3,1,3);
plot(xm_record1(:,3), 'g', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('z 방향 위치 [m]');
title('미사일 z 방향 위치 변화');
grid on;

figure;
subplot(3,1,1);
plot(xm_record1(:,4), 'b', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('x 방향 속도 [m/s]');
title('미사일 x 방향 속도 변화');
grid on;
subplot(3,1,2);
plot(xm_record1(:,5), 'r', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('y 방향 속도 [m/s]');
title('미사일 y 방향 속도 변화');
grid on;
subplot(3,1,3);
plot(xm_record1(:,6), 'g', 'LineWidth', 2);
xlabel('시간 [s]');
ylabel('z 방향 속도 [m/s]');
title('미사일 z 방향 속도 변화');
grid on;

% 가속도 계산
acc_x = diff(xm_record1(:,4)) / dt;
acc_y = diff(xm_record1(:,5)) / dt;
acc_z = diff(xm_record1(:,6)) / dt;
figure;
subplot(3,1,1);
plot(acc_x, 'b', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('x 가속도 [m/s²]');
title('미사일 x 방향 가속도 변화');
grid on;
subplot(3,1,2);
plot(acc_y, 'r', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('y 가속도 [m/s²]');
title('미사일 y 방향 가속도 변화');
grid on;
subplot(3,1,3);
plot(acc_z, 'g', 'LineWidth', 2);
xlabel('시간 [s]'); ylabel('z 가속도 [m/s²]');
title('미사일 z 방향 가속도 변화');
grid on;