function visualize_los(radar_pos, X, Y, Z, interval, visual_range)
    % radar_pos: 레이더 위치 [x, y, z]
    % X, Y, Z: 지형 데이터
    % interval: LOS 계산 간격
    % visual_range: 가시 거리 제한

    [grid_rows, grid_cols] = size(Z);
    visibility_map = zeros(grid_rows, grid_cols); % 0: 보임, 1: 안 보임

    % 가시성 계산
    for gx = 1:grid_rows
        for gy = 1:grid_cols
            % 목표 지점: 지형 고도 + 100
            target_pos = [X(gx, gy), Y(gx, gy), Z(gx, gy) + 100];
            
            % 레이더와 목표 지점 간 거리 계산
            range = norm(radar_pos - target_pos);
            if range > visual_range
                continue; % 시야 범위를 넘어가는 셀은 계산하지 않음
            end
            
            % LOS 계산 및 가시성 판단
            is_blocked = los_block_simple(radar_pos, target_pos, X, Y, Z, interval);
            if ~is_blocked
                visibility_map(gx, gy) = 0; % 보임 (녹색)
            else
                visibility_map(gx, gy) = 1; % 안 보임 (파란색)
            end
        end
    end

    % 시각화
    figure;
    hold on;
    surf(X / 1000, Y / 1000, Z, 'EdgeColor', 'none', 'FaceAlpha', 0.5); % 지형
    colormap([0 1 0; 0 0 1]); % 녹색(보임), 파란색(안 보임)
    scatter3(radar_pos(1) / 1000, radar_pos(2) / 1000, radar_pos(3), 100, 'r', 'filled'); % 레이더 위치
    imagesc(X(1, :) / 1000, Y(:, 1) / 1000, visibility_map); % 가시성 맵
    xlabel('X [km]');
    ylabel('Y [km]');
    zlabel('Altitude [m]');
    title('LOS Visibility');
    colorbar('Ticks', [0, 1], 'TickLabels', {'Visible', 'Blocked'});
    view(3);
    grid on;
end

function is_blocked = los_block_simple(radar_pos, target_pos, X, Y, Z, interval)
    % radar_pos: 레이더 위치 [x, y, z]
    % target_pos: 목표 위치 [x, y, z]
    % X, Y, Z: 지형 데이터
    % interval: LOS 계산 간격
    
    is_blocked = false;

    % LOS 벡터 계산
    delta = (target_pos - radar_pos) / interval;
    for step = 1:interval
        % 현재 LOS 지점
        current_pos = radar_pos + step * delta;

        % 2D 배열에서 근접한 셀의 인덱스 계산
        [gx, gy] = find_nearest_grid(current_pos(1), current_pos(2), X, Y, size(Z));

        % 고도 비교: 현재 지점의 고도와 LOS의 z 좌표 비교
        if Z(gx, gy) > current_pos(3)
            is_blocked = true; % 시야가 차단됨
            return;
        end
    end
end

function [gx, gy] = find_nearest_grid(x, y, X, Y, grid_size)
    % x, y 좌표에 해당하는 가장 가까운 그리드 인덱스를 계산
    [~, gx] = min(abs(X(1, :) - x));
    [~, gy] = min(abs(Y(:, 1) - y));
    
    % 인덱스가 배열 경계를 초과하지 않도록 제한
    gx = max(1, min(grid_size(1), gx));
    gy = max(1, min(grid_size(2), gy));
end