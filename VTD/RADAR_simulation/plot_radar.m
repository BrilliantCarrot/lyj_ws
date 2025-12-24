clear; clc; close all;
% load C:/Users/ThinkYun/lab_ws/data/VTD/Radar/MAP_STRUCT;
load C:/Users/leeyj/lab_ws/data/VTD/Radar/MAP_STRUCT.mat;

% dm = 샘플링 간격, 10
dm = 10; g_size = size(MAP.X,1);
mesh_x = floor(g_size/2)-250:dm:g_size-750;
mesh_y = floor(g_size/2)-370:dm:g_size-540;

X1 = MAP.X(mesh_x,mesh_y);
Y1 = MAP.Y(mesh_x,mesh_y);
X = X1 - min(min(X1));
Y = Y1 - Y1(1,1);
Z = MAP.alt(mesh_x,mesh_y);

load C:/Users/leeyj/lab_ws/data/VTD/Radar/Results_2GHz.mat
RADAR.RCS1 = Sth;
RADAR.theta = theta;
RADAR.psi = psi;
load C:/Users/leeyj/lab_ws/data/VTD/Radar/Results_8GHz.mat
RADAR.RCS2 = Sth;

x0 = 34000; y0 = 37400;
[ix, iy] = pos2grid(x0,y0,X,Y);
% ix = 80; iy = 40;
x = X(ix,iy); y = Y(ix,iy); z =  Z(ix,iy);
dt = 0.1; % 궤적 샘플링 주기, 0.1
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
    alt = 100;
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
% 처음 interval, visual_range는 30; 100000;
interval = 30; visual_range = 100000;
LOS_length = 5000; % LOS벡터의 뒤로 연장하여 지형지물 가림을 비교할 거리, m
% visibility_results = false(size(traj, 1), 1); % 가시성 결과 저장
block_check = false;
for i = 1:10:length(traj)
    hx = traj(i,1); hy = traj(i,2); hz = traj(i,3);
    visual_matrix = zeros(length(mesh_x),length(mesh_y));

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

            space = 100;
            RADAR.RadarPos(1,:) = [grid_x grid_y cal_alt(grid_x,grid_y,X,Y,Z)];
            % RADAR: 레이더 구조체
            % hx, hz, hz: 항공기 위치 좌표
            % X, Y, Z: 지표 행렬
            block_check = check_target_behind(RADAR,[hx,hy,hz],X,Y,Z,space,LOS_length);
            sig1 = RADAR_Module_SIR(block_check,RADAR,[hx,hy,hz],1,X,Y,Z);

            % sig1 = 4.5*(sig1-70);
            % sig_save(j,k) = sig1;

            % if sig < 90
            %     if sig < 0 
            %         sig = 0;
            %     end
            %     C(j,k,1) = 0; % C에 색깔 정보(가시 유무)가 저장됨
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
            % if visibility == 1
            %     C(j,k,1) = 0.5;
            %     C(j,k,2) = 0.5;
            %     C(j,k,3) = 0.5;
            % end
            
            if visibility == 1 % 기체가 보이지 않는 영역은 회색으로
                sig_save(j,k) = 0;
            else
                sig_save(j,k) = sig1;
            end
        end
    end
    % Visual_Struct{i} = visual_matrix;
    RADAR_sig_SIR{i} = sig_save;

    % RADAR_C_SIR{i} = C;
    % RADAR_C{i} = C; % 가시 여부가 반영된 신호 칼라맵
end

%% 시각화

figure(1)
clf
pause(1)
for i = 1:10:length(traj)                                                           
    s = surf(X/1000, Y/1000, Z, real(RADAR_sig_SIR{i})); 
    hold on;
    plot3(traj(1:i,1)/1000, traj(1:i,2)/1000, traj(1:i,3), '-', 'Color', 'k', 'LineWidth', 2); 
    hold on; 
    grid on;
    xlabel('X [km]');
    ylabel('Y [km]');
    zlabel('Altitude [m]');
    alpha(s, 0.5);
    view(-20, 85);
    pause(1);
    if i < length(traj)
        delete(s);
    end
end