function visibility_matrix = LOS_test_new(radar_pos, X, Y, Z)
    % radar_pos: [x, y, z] 레이더 좌표
    % X, Y, Z: 지형 데이터 (격자 형태)
    % visibility_matrix: 가시성 행렬 (1 = 보임, 0 = 안 보임)
    radar_x = radar_pos(1);
    radar_y = radar_pos(2);
    radar_z = radar_pos(3);
    visibility_matrix = zeros(size(Z));
    % 삼각 보간법을 위한 삼각형 설정
    F = scatteredInterpolant(X(:), Y(:), Z(:), 'linear', 'none'); % 선형 보간 사용
    for i = 1:size(Z, 1)    
        for j = 1:size(Z, 2)
            target_x = X(i, j);
            target_y = Y(i, j);
            target_z = Z(i, j)+30;
            num_steps = max(abs(target_x - radar_x), abs(target_y - radar_y));
            los_x = linspace(radar_x, target_x, num_steps);
            los_y = linspace(radar_y, target_y, num_steps);
            los_z = linspace(radar_z, target_z, num_steps);
            is_visible = true;
            for k = 2:num_steps-1
                terrain_z_interp = F(los_x(k), los_y(k)); % 삼각 보간법을 통한 지형 높이 추정
                if isnan(terrain_z_interp) % 보간이 불가능한 경우 패스
                    continue;
                end
                if terrain_z_interp > los_z(k)
                    is_visible = false;
                    break;
                end
            end
            visibility_matrix(i, j) = is_visible;
        end
    end
end