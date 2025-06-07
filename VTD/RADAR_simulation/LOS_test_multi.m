function visibility_matrix = LOS_test_multi(radars, X, Y, Z)
    % 복수의 레이더에 대한 가시성을 계산하여 가시성 행렬 산출
    % radars: 복수의 레이더 위치 (Nx3 행렬, 각 행이 레이더 위치)
    % radar_pos: [x, y, z] 레이더 좌표
    % X, Y, Z: 지형 데이터 (격자 형태)
    % visibility_matrix: 가시성 행렬 (1 = 보임, 0 = 안 보임)
    num_radars = size(radars, 1);
    visibility_matrix = zeros(size(Z));
    F = scatteredInterpolant(X(:), Y(:), Z(:), 'linear', 'none');
    for r = 1:num_radars
        radar_pos = radars(r,:);
        radar_x = radar_pos(1);
        radar_y = radar_pos(2);
        radar_z = radar_pos(3);
        for i = 1:size(Z, 1)    
            for j = 1:size(Z, 2)
                target_x = X(i, j);
                target_y = Y(i, j);
                target_z = Z(i, j)+100;
                num_steps = max(abs(target_x - radar_x), abs(target_y - radar_y));
                los_x = linspace(radar_x, target_x, num_steps);
                los_y = linspace(radar_y, target_y, num_steps);
                los_z = linspace(radar_z, target_z, num_steps);
                is_visible = true;
                for k = 2:num_steps-1
                    terrain_z_interp = F(los_x(k), los_y(k));
                    if isnan(terrain_z_interp)
                        continue;
                    end
                    if terrain_z_interp > los_z(k)
                        is_visible = false;
                        break;
                    end
                end
                visibility_matrix(i, j) = visibility_matrix(i, j) || is_visible;
            end
        end
    end
end