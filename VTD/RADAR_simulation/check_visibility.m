function visibility_map = check_visibility(radars, X, Y, Z, interval, visual_range)
    % radars: Nx3 행렬, 각 행이 레이더 위치
    % X, Y, Z: 지형 데이터
    % interval: LOS 계산 간격
    % visual_range: 시야 제한 거리
    
    [num_radars, ~] = size(radars);
    [grid_rows, grid_cols] = size(Z);
    visibility_map = zeros(grid_rows, grid_cols); % 0: 보임, 1: 안 보임
    
    for i = 1:num_radars
        radar_pos = radars(i, :);
        
        for gx = 1:grid_rows
            for gy = 1:grid_cols
                % 목표 지점
                target_pos = [X(gx, gy), Y(gx, gy), Z(gx, gy) + 100];
                
                % 레이더와 목표지점 사이 거리 계산
                range = norm(radar_pos - target_pos);
                if range > visual_range
                    continue; % 가시 범위를 넘어가면 계산하지 않음
                end

                % LOS 계산 및 가시성 판단
                is_blocked = los_block_simple(radar_pos, target_pos, X, Y, Z, interval);
                if ~is_blocked
                    visibility_map(gx, gy) = 0; % 보임
                else
                    visibility_map(gx, gy) = 1; % 안 보임
                end
            end
        end
    end
end

function is_blocked = los_block_simple(radar_pos, target_pos, X, Y, Z, interval)
    % radar_pos: 레이더 위치 [x, y, z]
    % target_pos: 목표 위치 [x, y, z]
    % X, Y, Z: 지형 데이터
    % interval: LOS 계산 간격
    
    is_blocked = false;
    [grid_rows, grid_cols] = size(Z);
    
    % LOS 벡터 계산
    delta = (target_pos - radar_pos) / interval;
    for step = 1:interval
        % 현재 LOS 지점
        current_pos = radar_pos + step * delta;

        % 2D 배열에서 근접한 셀의 인덱스 계산
        [gx, gy] = find_nearest_grid(current_pos(1), current_pos(2), X, Y, grid_rows, grid_cols);

        % 고도 비교: 현재 지점의 고도와 LOS의 z 좌표 비교
        if Z(gx, gy) > current_pos(3)
            is_blocked = true; % 시야가 차단됨
            return;
        end
    end
end

function [gx, gy] = find_nearest_grid(x, y, X, Y, grid_rows, grid_cols)
    % x, y 좌표에 해당하는 가장 가까운 그리드 인덱스를 계산
    gx = max(1, min(grid_rows, find(abs(X(1, :) - x) == min(abs(X(1, :) - x), [], 'all'), 1)));
    gy = max(1, min(grid_cols, find(abs(Y(:, 1) - y) == min(abs(Y(:, 1) - y), [], 'all'), 1)));
end

% function z = calculate_Z(x, y, X, Y, Z)
%     % x, y 좌표에 해당하는 Z 값을 계산
%     [~, ix] = min(abs(X(1, :) - x));
%     [~, iy] = min(abs(Y(:, 1) - y));
%     z = Z(iy, ix);
% end