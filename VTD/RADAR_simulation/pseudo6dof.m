%% Psuedo 6 DoF Simulation
% simulation for heliocopter case only
clc; 
clear; 
close all;

r_m0 = [0; 0; 0];
v_m0 = [200; 0; 0];
r_t0 = [5000; 3100; 300];
v_t0 = [0; 0; 0];
N = 4;
% 유도탄 위치, 속도 정의
r_m = r_m0;
v_m = v_m0;
% 표적 위치, 속도 정의
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

t = 0;
dt = 0.01;
t_step = t;
tspan = [0 30]; % 30초 동안 시뮬레이션
xm0 = [r_m0; v_m0];
xt0 = [r_t0; v_t0];

xm = [r_m0; v_m0];
xm_record1 = xm.';
xt = [r_t0; v_t0];
xt_record1 = xt.';

save_state = [0;0;0;0;0;0];
save_state_m = save_state;
save_state_t = save_state;
sum_k = [0;0;0;0;0;0];
sum_k_m = sum_k;
sum_k_t = sum_k;

%% ode 45를 통한 시뮬레이션 수행

[t_m, xm_record] = ode45(@(t, xm) missile_dynamics(t, xm, xt0, N), tspan, xm0);
[t_t, xt_record] = ode45(@(t, xt) target_dynamics(t, xt, xm0, N), tspan, xt0);

figure;
plot3(xm_record(:,1), xm_record(:,2), xm_record(:,3), 'b', 'LineWidth', 2);
hold on;
plot3(xt_record(:,1), xt_record(:,2), xt_record(:,3), 'r', 'LineWidth', 2);
plot3(r_m0(1), r_m0(2), r_m0(3), 'bp', 'MarkerSize', 10, 'LineWidth', 2);
plot3(r_t0(1), r_t0(2), r_t0(3), 'rp', 'MarkerSize', 10, 'LineWidth', 2);
legend({'미사일 궤적', '타겟 궤적', '미사일 초기 위치', '타겟 초기 위치'}, 'Location', 'northwest');
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
grid on;
title("PPN 적용 시뮬레이션 결과");
hold off;

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