function visibility_matrix = LOS_test_old(radar_pos, X, Y, Z)
    % radar_pos: [x, y, z] 의 레이더 좌표
    % X, Y, Z: 레이더가 바라보는 각 지형의 좌표 및 고도 
    % visibility_matrix: 가시성 유무가 담긴 행렬

    radar_1 = radar_pos;
    radar_x = radar_1(1);
    radar_y = radar_1(2);
    radar_z = radar_1(3);
    visibility_matrix = zeros(size(Z));
    for i = 1:size(Z, 1)    
        for j = 1:size(Z, 2)
            target_x = X(i,j);
            target_y = Y(i,j);
            target_z = Z(i,j);
            num_steps = max(abs(target_x - radar_x), abs(target_y - radar_y));
            los_x = linspace(radar_x, target_x, num_steps);
            los_y = linspace(radar_y, target_y, num_steps);
            los_z = linspace(radar_z, target_z, num_steps);
            is_visible = true;
            for k = 2:num_steps-1
                [~, closest_row] = min(abs(Y(:, 1) - los_y(k)));
                [~, closest_col] = min(abs(X(1, :) - los_x(k)));
                terrain_z = Z(closest_row, closest_col);
                if terrain_z > los_z(k)
                    is_visible = false;
                    break;
                end
            end
            visibility_matrix(i, j) = is_visible;
        end
    end
end