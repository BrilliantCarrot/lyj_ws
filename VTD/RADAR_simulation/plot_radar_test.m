%% MAP Initialize
clear; clc; close all;
load C:/Users/leeyj/lab_ws/data/VTD/Radar/MAP_STRUCT;

% dm = 샘플링 간격
dm = 30; g_size = size(MAP.X,1);
mesh_x = floor(g_size/2)-250:dm:g_size-750;
mesh_y = floor(g_size/2)-370:dm:g_size-540;

X1 = MAP.X(mesh_x,mesh_y);
Y1 = MAP.Y(mesh_x,mesh_y);
X = X1 - min(min(X1));
Y = Y1 - Y1(1,1);

Z = MAP.alt(mesh_x,mesh_y);

%% RADAR Initialize
load C:/Users/leeyj/lab_ws/data/VTD/Radar/Results_2GHz.mat
RADAR.RCS1 = Sth;
RADAR.theta = theta;
RADAR.psi = psi;
load C:/Users/leeyj/lab_ws/data/VTD/Radar/Results_8GHz.mat
RADAR.RCS2 = Sth;
% RADAR.RadarPos = zeros(size(131,164, 1), size(131,164, 2), 3);

% 레이더 파라미터 설정
% for lambda num = 1, 2GHz
% if lambda_num == 1
    % rcs_table = RADAR.RCS1;
    % pitch_array = RADAR.theta(1,:) * pi/180;
    % yaw_array = RADAR.psi(:,1) * pi/180;
    % % sig = 0;
    % lambda = freq2wavelen(2*10^9);          % [m] wavelength
    % Pt = 14000;                             % [W] peak power
    % tau = 0.00009;                          % [s] pulse width
    % G = 34;                                 % [dBi] antenna gain
    % Ts = 290;                               % [K] System temp 
    % L = 8.17;                                  % [dB] Loss
    % prf = 1000;                             % [Hz] Pulse repetition frequency 
% for lambda num = 2, 8GHz
    % elseif lambda_num == 2  
    % rcs_table = RADAR.RCS2;
    % pitch_array = RADAR.theta(1,:) * pi/180;
    % yaw_array = RADAR.psi(:,1) * pi/180;
    % % sig = 0;
    % lambda = freq2wavelen(8*10^9);          % [m] wavelength
    % Pt = 6000;                              % [W] peak power
    % tau = 0.0001;                           % [s] pulse width
    % G = 39;                                 % [dBi] antenna gain
    % Ts = 290;                               % [K] System temp 
    % L = 0;                                  % [dB] Loss
    % prf = 2200;                             % [Hz] Pulse repetition frequency 

% end

%% Trajectory
x0 = 34000; y0 = 37400;
[ix, iy] = pos2grid(x0,y0,X,Y);
% ix = 80; iy = 40;
x = X(ix,iy); y = Y(ix,iy); z =  Z(ix,iy);
dt = 1;   % 궤적 샘플링 주기, 0.1
vx = 0; vy = 0; vz = 0;
traj = [x y z vx vy vz];
k = 2;

for t = 0:dt:11
    vz = 10;
    if z > Z(ix,iy) + 100
        vz = 0;
    end
    x = x + vx*dt;
    y = y + vy*dt;
    z = z + vz*dt;   

    traj(k,:) = [x y z vx vy vz];
    k = k+1;

end
    alt = 100;  % 헬기 고도(100미티 만큼 상승 지형에서 떨어져서 기동)
    k = k-1;
for t = 11:dt:190
    vx = -3*60; vy = -3*60;
    x =  x+vx*dt;
    y =  y+vy*dt;
    z = alt + cal_alt(x,y,X,Y,Z);
    traj(k,:) = [x y z vx vy vz];
    k = k+1;
end

%% Cal Visibility
clc;
% 30; 100000;
interval = 60; visual_range = 80000;
LOS_length = 50000;
visibility_results = false(size(traj, 1), 1);               % 가시성 결과 저장
block_check = false;
for i = 1:10:length(traj)
    hx = traj(i,1); hy = traj(i,2); hz = traj(i,3);
    visual_matrix = zeros(length(mesh_x),length(mesh_y));
    
    % PosN = [hx,hy,hz];      % 현재 목표물 위치

    
    
    % is_blocked = false;     % 초기 상태: 시선벡터가 지형에 

    for j = 1:length(mesh_x)
        for k = 1:length(mesh_y)

            grid_x = X(j,k); grid_y = Y(j,k);
            if grid_x < 0.1
                grid_x = 0.1;
            end

            if grid_y < 0.1
                grid_y = 0.1;
            end


            if grid_x < hx
                dx = (hx - grid_x)/interval;
                LOSx = grid_x:dx:hx;
            elseif grid_x > hx
                dx = (-hx + grid_x)/interval;
                LOSx = hx:dx:grid_x;
            else
                LOSx = zeros(1,interval+1);
                LOSx(:) = hx;
            end

            if grid_y < hy
                dy = (hy - grid_y)/interval;
                LOSy = grid_y:dy:hy;
            elseif grid_y > hy
                dy = (-hy + grid_y)/interval;
                LOSy = hy:dy:grid_y;
            else
                LOSy = zeros(1,interval+1);
                LOSy(:) = hy;
            end

            threshold_z = max(cal_alt(hx,hy,X,Y,Z),cal_alt(grid_x,grid_y,X,Y,Z));
            
            check_point = 0;
            for check_idx = 1:interval+1
                

                minZ = min(cal_alt(hx,hy,X,Y,Z),cal_alt(grid_x,grid_y,X,Y,Z));
                maxZ = max(cal_alt(hx,hy,X,Y,Z),cal_alt(grid_x,grid_y,X,Y,Z));

                threshold_z = minZ + (maxZ-minZ)*check_idx/interval+1;
                check_alt = cal_alt(LOSx(check_idx),LOSy(check_idx),X,Y,Z);
                if check_alt > threshold_z
                    check_point = check_point + 1;
                    break;
                end
            end
            range = norm([hx-grid_x hy-grid_y]);
            if check_point == 0 && range < visual_range
                visibility = 0;
            else
                visibility = 1;
            end
%             visual_matrix(j,k) = visibility;


            RADAR.RadarPos(1,:) = [grid_x grid_y cal_alt(grid_x,grid_y,X,Y,Z)];
            % RPos_save(j,k,:) = RADAR.RadarPos(1,:);
            block_check = check_target_behind(RADAR,[hx,hy,hz],X,Y,Z,interval,LOS_length);
            % disp(RADAR.RadarPos(1,:));
            % sig1에 SIR_dB가 들어감
            [sig1,sigma_MBc,sigma_SLc,sigma_clutter,SNR,SCR,SIR_dB,Range] = RADAR_Module_SIR(block_check,RADAR,[hx,hy,hz],1,X,Y,Z);
            % [sig1,SNR,Range] = RADAR_Module_SNR(RADAR,[hx,hy,hz],1,X,Y,Z);

            % sig1 = 4.5*(sig1-70);

            sig_save(j,k) = sig1;
            % MBc_save(j,k) = sigma_MBc;
            % SLc_save(j,k) = sigma_SLc;
            % sigma_clutter_save(j,k) = sigma_clutter;
            SNR_save(j,k) = SNR;
            SCR_save(j,k) = SCR;
            SIR_save(j,k) = SIR_dB;
            Range_save(j,k) = Range;
            % pos_save(j,k) = RADAR.RadarPos(1,3);

            % if sig < 90
            %     if sig < 0 
            %         sig = 0;
            %     end
            %     C(j,k,1) = 0;           % C에 색깔 정보(가시 유무)가 저장됨
            %     C(j,k,2) = sind(sig);
            %     C(j,k,3) = cosd(sig);
            % else
            %     if sig  > 180
            %         sig = 180;
            %     end
            %     C(j,k,1) = sind(sig-90);
            %     C(j,k,2) = cosd(sig-90);
            %     C(j,k,3) = 0;
            % end
            % if visibility == 1  % 기체가 레이더에 안 보이게 된다면 회색으로 처리
            %     C(j,k,1) = 0.5;
            %     C(j,k,2) = 0.5;
            %     C(j,k,3) = 0.5;
            % end

            % if visibility == 1 | sig1 < 0 % 기체가 레이더에 안 보이게 된다면 회색으로 처리
            %     sig_save(j,k) = 0;
            % else
            %     sig_save(j,k) = sig1;
            % end
        end
    end
    % Visual_Struct{i} = visual_matrix;
    % RADAR_sig_SNR{i} = sig_save;    % 가시 여부를 고려 안하고 SIR만 적용된 원래 칼라맵
    RADAR_sig_SIR{i} = sig_save;
    % MBc_mat{i} = MBc_save;
    % SLc_mat{i} = SLc_save;
    % sigma_clutter_mat{i} = sigma_clutter_save;
    SNR_mat{i} = SNR_save;
    SCR_mat{i} = SCR_save;
    SIR_mat{i} = SIR_save;
    Range_mat{i} = Range_save;
    % RPos_mat{i} = RPos_save;
    % pos_mat{i} = pos_save;

    % RADAR_C_SIR{i} = C;
    % RADAR_C{i} = C;             % 가시 여부가 반영된 신호 칼라맵
end

%% 시각화

figure(1)
set(gcf, 'Position', [200, 100, 1000, 750]); % [left, bottom, width, height]
clf
pause(1)
for i = 1:10:length(traj)                                                           
    s = surf(X/1000, Y/1000, Z, RADAR_sig_SIR{i}); hold on;
    plot3(traj(1:i,1)/1000, traj(1:i,2)/1000, traj(1:i,3), '-', 'Color', 'k', 'LineWidth', 2); hold on; grid on;

    % 현재 목표물 위치 계산
    % target_pos = traj(i, 1:3) / 1000; % 현재 목표물 위치 (km 단위)
    % % 모든 지형 셀에서 목표물로의 LOS 벡터 계산 및 시각화
    % LOS_length = 30; % LOS 벡터 길0이 (단위: km)
    % num_points = 20; % 샘플링 점 수

    % 루프를 통해 각 지형 셀에서 LOS 벡터 생성
    % for row = 1:size(X, 1)
    %     for col = 1:size(X, 2)
    %         % 현재 셀의 레이더 위치
    %         radar_pos = [X(row, col), Y(row, col), Z(row, col)] / 1000; % (km 단위)
    % 
    %         % LOS 벡터 계산
    %         LOS_direction = (target_pos - radar_pos) / norm(target_pos - radar_pos); % 단위 벡터
    %         LOS_points = zeros(num_points, 3);
    %         for j = 1:num_points
    %             LOS_points(j, :) = radar_pos + (j * (LOS_length / num_points)) * LOS_direction;
    %         end
    % 
    %         % LOS 벡터 시각화
    %         plot3(LOS_points(:,1), LOS_points(:,2), LOS_points(:,3), '-', 'Color', 'r', 'LineWidth', 0.5);
    %     end
    % end

    xlabel('X [km]');
    ylabel('Y [km]');
    zlabel('Altitude [m]');
    alpha(s, 0.5);
    view(-20, 80);
    clim([min(RADAR_sig_SIR{i}(:)), max(RADAR_sig_SIR{i}(:))]);
    c = colorbar;
    c.Label.String = 'RADAR Signal (SIR in dB)';
    pause(1);
    if i < length(traj)
        delete(s);
    end
end