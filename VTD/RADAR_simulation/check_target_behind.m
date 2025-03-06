function is_blocked = check_target_behind(RADAR, PosN, X,Y,Z,space,LOS_length)
    % 레이더 -> 기체 LOS벡터를 뒤로 연장하여 지형에 의해 차단되는지 유무를 검사
    % RADAR: 레이더 구조체(위치 포함)
    % PosN: 목표물 위치 [x, y, z]
    % X, Y, Z: 지형 데이터 (각각 x, y, z 좌표 행렬)
    % space: 지형의 막힘 유무 판별시 LOS 벡터 샘플링 간격
    % LOS_length: 목표물의 뒤로 연장될 LOS 벡터 길이 (단위: m)
    
    is_blocked = false; % 막힘 판단 유무 변수, 기본적으로 배경이 하늘이다 가정
    radar_pos = RADAR.RadarPos(1, :); % 레이더 위치 [x, y, z]
    radar_x = radar_pos(1);
    radar_y = radar_pos(2);
    radar_z = radar_pos(3);
    target_x = PosN(1);
    target_y = PosN(2);
    target_z = PosN(3);
    % 기본 LOS 벡터
    los_x = linspace(radar_x, target_x, space);
    los_y = linspace(radar_y, target_y, space);
    los_z = linspace(radar_z, target_z, space);
    % LOS를 목표물 뒤로 연장
    target_ext = PosN + LOS_length * (PosN - radar_pos) / norm(PosN - radar_pos);
    los_x_ext = linspace(target_x, target_ext(1), 100);
    los_y_ext = linspace(target_y, target_ext(2), 100);
    los_z_ext = linspace(target_z, target_ext(3), 100);
    % LOS_direction = (PosN - radar_pos) / norm(PosN - radar_pos); % 단위 LOS 벡터
    % num_points = floor(LOS_length / space);
    % 3 차원 보간 수행
    F = scatteredInterpolant(X(:), Y(:), Z(:), 'linear', 'none');

    for k = 1:length(los_x_ext)
        terrain_z = F(los_x_ext(k), los_y_ext(k));
        % 지형 고도가 연장된 LOS 벡터의 고도보다 높다면 "지형에 막힘"으로 판정
        if ~isnan(terrain_z) && terrain_z > los_z_ext(k)
            is_blocked = true;
            break;
        end
    end


    % for i = 1:num_points
        % sample_pos = PosN + (i * space) * LOS_direction; % 샘플링된 LOS 점
        % terrain_height = F(sample_pos(1), sample_pos(2));
        % if isnan(terrain_height) % 보간이 불가능한 경우 패스
        %     continue;
        % end
        % 
        % % 현재 샘플링 지점이 지형보다 낮은지 확인
        % if sample_pos(3) < terrain_height
        %     is_blocked = true;
        %     return;
        % end
        % x = current_point(1);
        % y = current_point(2);
        % z = current_point(3);
        


        
        % % 현재 점에 가장 가까운 지형 셀의 인덱스 계산
        % [~, ix] = min(abs(X(1,:) - x)); % x축에서 가장 가까운 셀
        % [~, iy] = min(abs(Y(:,1) - y)); % y축에서 가장 가까운 셀
        % % 지형 고도 가져오기
        % if ix > 0 && ix <= size(X, 2) && iy > 0 && iy <= size(Y, 1)
        %     terrain_alt = Z(iy, ix);
        %     % 지형 고도가 LOS 벡터의 고도보다 높으면 가려짐으로 판단
        %     if terrain_alt > z
        %         is_blocked = true; % 가려짐으로 설정
        %         % fprintf("지형에 의해 가려짐 \n")
        %         break; % 더 이상 확인하지 않음
        %     end
        %     % fprintf("지형에 의해 가려지지 않음 \n")
        % end
    % end
end

