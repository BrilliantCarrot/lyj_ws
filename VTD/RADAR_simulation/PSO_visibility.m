% 5 방향 추가 탐색하는 PSO
% function [optimal_path, sir_values, visibility_values] = PSO_visibility(radars, start_pos, end_pos, X, Y, Z, RADAR, visibility_matrix)
%     % PSO 알고리즘을 정의한 함수
%     % 레이더 피탐성이 최소가 되는 영역을 찾도록 함
%     % 첫 번째 매개변수: 단일 레이더 좌표 radar_pos 혹은
%     % 복수의 레이더(radars) 좌표
%     % start_pos: 시작점 위치 [x, y, z]
%     % end_pos: 목표점 위치 [x, y, z]
%     % X, Y, Z: 지형 데이터
%     % RADAR: 레이더 구조체
%     num_particles = 500;   % 입자 수 줄임 (더 작은 탐색 영역)
%     max_iter = 10;         % 반복 수
%     w = 0.7;               % 관성 계수
%     c1 = 1.5;              % 개인 가속 계수
%     c2 = 1.5;              % 전역 가속 계수
%     initial_sir = find_sir_multi(radars, start_pos, RADAR, X, Y, Z);  % 시작점의 SIR 계산
%     optimal_path = [start_pos, initial_sir];
%     % optimal_path = start_pos;
%     current_point = start_pos;
% 
%     lookahead_depth  = 2;      % 2스텝까지 미리 본다
%     lookahead_weight = 0.5;    % 룩어헤드 누적 비용 가중치
%     % sir_data = {};
% 
%     search_radius = 500;
%     max_search_radius = 2000; % 정체 시 탐색 반경 증가
%     min_distance = 30;
%     max_stagnation = 3;
%     stagnation_count = 0;
%     previous_gbest_score = inf;
% 
%     sir_values = []; % gbest위치의 sir을 저장할 빈 배열
%     visibility_values = [];
%     F = scatteredInterpolant(X(:), Y(:), Z(:), 'linear', 'none'); % 고도 추출을 위한 선형 보간
% 
%     while norm(current_point - end_pos) > min_distance
%         % 입자의 위치와 속도 초기화
%         particles = initialize_particles(current_point, end_pos, num_particles, search_radius, X, Y, Z,F);
%         velocities = zeros(size(particles));
%         % 초기 개인 최적값 및 전역 최적값 설정
%         pbest = particles;
%         pbest_scores = arrayfun(@(i) calculate_fitness(radars, particles(i, :), end_pos, ...
%             visibility_matrix, RADAR, X, Y, Z,lookahead_depth, lookahead_weight, search_radius), 1:num_particles)';
%         [gbest_score, gbest_idx] = min(pbest_scores);
%         gbest = pbest(gbest_idx, :);
%         % PSO 반복문
%         % 반복문을 돌며 개별 입자 전체의 SIR 계산
%         for iter = 1:max_iter
%             % fprintf("%d 번째 반복입니다 \n.", iter);
%             for i = 1:num_particles
%                 % 현재 입자의 SIR 계산
%                 current_score = calculate_fitness(radars, particles(i, :), end_pos, ...
%                     visibility_matrix, RADAR, X, Y, Z, lookahead_depth, lookahead_weight, search_radius);
%                 % 개인 최적값 업데이트
%                 if current_score < pbest_scores(i)
%                     pbest_scores(i) = current_score;
%                     pbest(i, :) = particles(i, :);
%                 end
%                 % 전역 최적값 업데이트
%                 if current_score < gbest_score
%                     gbest_score = current_score;
%                     gbest = particles(i, :);
%                 end
%             end
%             % 속도 및 위치 업데이트
%             for i = 1:num_particles
%                 r1 = rand();
%                 r2 = rand();
%                 velocities(i, :) = w * velocities(i, :) + ...
%                                    c1 * r1 * (pbest(i, :) - particles(i, :)) + ...
%                                    c2 * r2 * (gbest - particles(i, :));
%                 particles(i, :) = particles(i, :) + velocities(i, :);
%                 % 업데이트된 입자의 위치를 탐색 반경 내로 제한하여 생성되도록 함
%                 particles(i, :) = constrain_to_radius(current_point, particles(i, :), search_radius, X, Y, Z,F);
%             end
%             % 정체 상황(경로 계획이 막힘) 처리: 정체 시 탐색 반경 확장
%             if stagnation_count >= max_stagnation
%                 % for i = 1:num_particles
%                 %     random_offset = (rand(1, 3) - 0.5) * search_radius*3; % 랜덤 오프셋
%                 %     particles(i, :) = gbest + random_offset; % 전역 최적점 주변으로 흩뿌림
%                 %     particles(i, 3) = calculate_Z(particles(i, 1), particles(i, 2), X, Y, Z) + 30; % 고도 보정
%                 % end
%                 fprintf('Stagnation detected! Expanding search radius and forcing movement.\n');
%                 search_radius = min(search_radius * 2, max_search_radius);
%                 best_alternative_idx = find(pbest_scores == min(pbest_scores), 1);
%                 gbest = pbest(best_alternative_idx, :); % 현재 위치에서 가장 좋은 대안으로 이동
%                 stagnation_count = 0; % 정체 카운트 초기화
%             end
%         end
%         % 다음 경로 점 업데이트
%         current_point = gbest;
%         % 전역 최적 점 위치에서의 SIR 계산 및 저장
%         current_sir = find_sir_multi(radars, current_point, RADAR, X, Y, Z);
%         sir_values = [sir_values; current_sir];
%         % 경로점 시퀀스 저장
%         optimal_path = [optimal_path; current_point, current_sir];
% 
%         % 현재 경로점(gbest)의 가시성 여부 체크
%         [~, ix] = min(abs(X(1, :) - gbest(1)));
%         [~, iy] = min(abs(Y(:, 1) - gbest(2)));
% 
%         visibility_status = visibility_matrix(iy, ix);  % 0 = 안 보임, 1 = 보임
%         visibility_values = [visibility_values; visibility_status];  % 경로점 가시성 기록
% 
% 
%         % % 현재 지형에서의 SIR 데이터 저장
%         % sir_matrix = zeros(size(Z));
%         % for i = 1:size(Z, 1)
%         %     for j = 1:size(Z, 2)
%         %         target_pos = [X(i, j), Y(i, j), Z(i, j)];
%         %         % 복수의 레이더의 경우 find_sir_multi 함수를 호출하여 모든 레이더 마다의 SIR을 계산
%         %         sir_matrix(i, j) = find_sir_multi(radars, target_pos, RADAR, X, Y, Z);
%         %     end
%         % end
%         % sir_data{end + 1} = sir_matrix;
% 
%         if abs(previous_gbest_score - gbest_score) < 1e-3
%             stagnation_count = stagnation_count + 1;
%         else
%             stagnation_count = 0;
%             search_radius = 500;
%         end
%         previous_gbest_score = gbest_score;
%         fprintf('Current Point: (X: %.2f, Y: %.2f, Z: %.2f), Best Fitness: %.2f, Stagnation: %d\n', ...
%                 current_point(1), current_point(2), current_point(3), gbest_score, stagnation_count);
%     end
% end
% % 입자 초기화 함수
% function particles = initialize_particles(current_point, end_point, num_particles, radius, X, Y, Z,F)
%     particles = zeros(num_particles, 3);
%     direction = (end_point - current_point) / norm(end_point - current_point); % 방향 벡터 계산
%     for i = 1:num_particles
%         offset = randn(1, 3) * radius; % 랜덤 오프셋 생성
%         offset(3) = 0; % 수평 방향으로만 랜덤 오프셋을 추가
%         particles(i, :) = current_point + offset + direction * radius * rand(); % 방향성을 추가한 초기화
%         particles(i, 3) = calculate_Z(particles(i, 1), particles(i, 2), X, Y, Z,F) + 30; % 고도 보정
%     end
% end
% % 탐색 반경 내로 위치 제한
% % 입자가 탐색 반경을 벗어나는 경우 반경 내 가장자리로 제한
% function position = constrain_to_radius(center, position, radius, X, Y, Z,F)
%     if norm(position - center) > radius
%         direction = (position - center) / norm(position - center);
%         position = center + direction * radius;
%     end
%     position(3) = calculate_Z(position(1), position(2), X, Y, Z,F) + 30; % 고도 보정
% end
% % x,y 좌표에서 맞는 고도 z값을 산출
% function z = calculate_Z(x, y, X, Y, Z, F)
%     [~, ix] = min(abs(X(1, :) - x));
%     [~, iy] = min(abs(Y(:, 1) - y));
%     z = Z(iy, ix);
%     % 보간 적용 테스트
%     % z = F(x, y);
%     % if isnan(z)
%     %     distances = sqrt((X(:) - x).^2 + (Y(:) - y).^2);
%     %     [~, idx] = min(distances);
%     %     z = Z(idx);
%     % end
% end
% % 적합도(fitness) 계산 함수
% % SIR이 최소화하는 것과 동시에 목표점까지 직선 거리를 최대한 유지하도록 설계
% % 복수의 레이더 경우 find_sir_multi 적용
% function fitness = calculate_fitness(radars, particle_pos, end_pos, ...
%     visibility_matrix, RADAR, X, Y, Z, lookahead_depth, lookahead_weight, search_radius)
%     sir_value = find_sir_multi(radars, particle_pos, RADAR, X, Y, Z);
%     distance_to_goal = norm(particle_pos - end_pos);
%     [~, ix] = min(abs(X(1, :) - particle_pos(1)));
%     [~, iy] = min(abs(Y(:, 1) - particle_pos(2)));
%     % 보간 적용 테스트
%     % distances = (X - particle_pos(1)).^2 + (Y - particle_pos(2)).^2;
%     % [~, minIdx] = min(distances(:));
%     % [iy, ix] = ind2sub(size(X), minIdx);
%     visibility_bonus = 0;
%     if visibility_matrix(iy, ix) == 0  % 가려진 영역이면 보상 부여
%         visibility_bonus = -10;
%     end
% 
%     fitness = sir_value + 0.01 * distance_to_goal + visibility_bonus;
% 
%     % --- 룩어헤드 누적 적합도 계산 ---
%     lookahead_cost = compute_lookahead_cost(radars, particle_pos, end_pos, visibility_matrix, ...
%         RADAR, X, Y, Z, lookahead_depth, lookahead_weight, search_radius);
% 
%     % --- 최종 fitness ---
%     fitness = fitness + lookahead_weight * lookahead_cost;
% end
% 
% 
% 
% 
% % 재귀 혹은 반복으로 구현하는 룩어헤드 계산 함수
% function cost = compute_lookahead_cost(radars, cur_pos, end_pos, vis_mat, RADAR, X, Y, Z, depth, lookahead_weight, search_radius)
%     if depth == 0
%         cost = 0;
%         return
%     end
% 
%     % 목표 방향으로의 unit vector
%     dir_vec = (end_pos - cur_pos);
%     dir_vec = dir_vec / norm(dir_vec);
% 
%     % 각 스텝마다 시도해볼 이동 방향들 (여기선 5방향 예시)
%     angles = [ 0,  45, -45,  90, -90 ] * pi/180;  % rad
%     candidates = zeros(numel(angles), 3);
%     for k = 1:numel(angles)
%         Rz = [cos(angles(k)) -sin(angles(k)) 0; sin(angles(k)) cos(angles(k)) 0; 0 0 1];
%         offset_dir = (Rz * dir_vec')';
%         candidates(k,:) = cur_pos + offset_dir * search_radius;  
%         % 고도 보정
%         candidates(k,3) = calculate_Z(candidates(k,1), candidates(k,2), X, Y, Z);
%     end
% 
%     % 각 후보에 대한 즉시 적합도 + 다음 스텝 룩어해드 비용
%     costs = zeros(numel(angles),1);
%     for k = 1:numel(angles)
%         c_pos = candidates(k,:);
%         % 즉시 적합도
%         sir_v = find_sir_multi(radars, c_pos, RADAR, X, Y, Z);
%         d2g   = norm(c_pos - end_pos);
%         [~, ix] = min(abs(X(1,:) - c_pos(1)));
%         [~, iy] = min(abs(Y(:,1) - c_pos(2)));
%         vis_b  = (vis_mat(iy,ix)==0) * (-10);
%         immed = sir_v + 0.01 * d2g + vis_b;
% 
%         % 재귀 호출로 다음 스텝 비용
%         future = compute_lookahead_cost(radars, c_pos, end_pos, vis_mat, RADAR, X, Y, Z, depth-1, lookahead_weight, search_radius);
% 
%         costs(k) = immed + lookahead_weight * future;
%     end
% 
%     % 후보들 중 최소값 반환
%     cost = min(costs);
% end


% 원래 PSO
function [optimal_path, sir_values, visibility_values] = PSO_visibility(radars, start_pos, end_pos, X, Y, Z, RADAR, visibility_matrix)
    % PSO 알고리즘을 정의한 함수
    % 레이더 피탐성이 최소가 되는 영역을 찾도록 함
    % 첫 번째 매개변수: 단일 레이더 좌표 radar_pos 혹은
    % 복수의 레이더(radars) 좌표
    % start_pos: 시작점 위치 [x, y, z]
    % end_pos: 목표점 위치 [x, y, z]
    % X, Y, Z: 지형 데이터
    % RADAR: 레이더 구조체
    num_particles = 500;   % 입자 수 줄임 (더 작은 탐색 영역)
    max_iter = 10;         % 반복 수
    w = 0.7;               % 관성 계수
    c1 = 1.5;              % 개인 가속 계수
    c2 = 1.5;              % 전역 가속 계수
    initial_sir = find_sir_multi(radars, start_pos, RADAR, X, Y, Z);  % 시작점의 SIR 계산
    optimal_path = [start_pos, initial_sir];
    % optimal_path = start_pos;
    current_point = start_pos;


    % sir_data = {};

    search_radius = 500;
    max_search_radius = 2000; % 정체 시 탐색 반경 증가
    min_distance = 30;
    max_stagnation = 3;
    stagnation_count = 0;
    previous_gbest_score = inf;

    sir_values = []; % gbest위치의 sir을 저장할 빈 배열
    visibility_values = [];
    F = scatteredInterpolant(X(:), Y(:), Z(:), 'linear', 'none'); % 고도 추출을 위한 선형 보간

    while norm(current_point - end_pos) > min_distance
        % 입자의 위치와 속도 초기화
        particles = initialize_particles(current_point, end_pos, num_particles, search_radius, X, Y, Z,F);
        velocities = zeros(size(particles));
        % 초기 개인 최적값 및 전역 최적값 설정
        pbest = particles;
        pbest_scores = arrayfun(@(i) calculate_fitness(radars, particles(i, :), end_pos, visibility_matrix, RADAR, X, Y, Z), 1:num_particles)';
        [gbest_score, gbest_idx] = min(pbest_scores);
        gbest = pbest(gbest_idx, :);
        % PSO 반복문
        % 반복문을 돌며 개별 입자 전체의 SIR 계산
        for iter = 1:max_iter
            % fprintf("%d 번째 반복입니다 \n.", iter);
            for i = 1:num_particles
                % 현재 입자의 SIR 계산
                current_score = calculate_fitness(radars, particles(i, :), end_pos, visibility_matrix, RADAR, X, Y, Z);
                % 개인 최적값 업데이트
                if current_score < pbest_scores(i)
                    pbest_scores(i) = current_score;
                    pbest(i, :) = particles(i, :);
                end
                % 전역 최적값 업데이트
                if current_score < gbest_score
                    gbest_score = current_score;
                    gbest = particles(i, :);
                end
            end
            % 속도 및 위치 업데이트
            for i = 1:num_particles
                r1 = rand();
                r2 = rand();
                velocities(i, :) = w * velocities(i, :) + ...
                                   c1 * r1 * (pbest(i, :) - particles(i, :)) + ...
                                   c2 * r2 * (gbest - particles(i, :));
                particles(i, :) = particles(i, :) + velocities(i, :);
                % 업데이트된 입자의 위치를 탐색 반경 내로 제한하여 생성되도록 함
                particles(i, :) = constrain_to_radius(current_point, particles(i, :), search_radius, X, Y, Z,F);
            end
            % 정체 상황(경로 계획이 막힘) 처리: 정체 시 탐색 반경 확장
            if stagnation_count >= max_stagnation
                % for i = 1:num_particles
                %     random_offset = (rand(1, 3) - 0.5) * search_radius*3; % 랜덤 오프셋
                %     particles(i, :) = gbest + random_offset; % 전역 최적점 주변으로 흩뿌림
                %     particles(i, 3) = calculate_Z(particles(i, 1), particles(i, 2), X, Y, Z) + 30; % 고도 보정
                % end
                fprintf('Stagnation detected! Expanding search radius and forcing movement.\n');
                search_radius = min(search_radius * 2, max_search_radius);
                best_alternative_idx = find(pbest_scores == min(pbest_scores), 1);
                gbest = pbest(best_alternative_idx, :); % 현재 위치에서 가장 좋은 대안으로 이동
                stagnation_count = 0; % 정체 카운트 초기화
            end
        end
        % 다음 경로 점 업데이트
        current_point = gbest;
        % 전역 최적 점 위치에서의 SIR 계산 및 저장
        current_sir = find_sir_multi(radars, current_point, RADAR, X, Y, Z);
        sir_values = [sir_values; current_sir];
        % 경로점 시퀀스 저장
        optimal_path = [optimal_path; current_point, current_sir];

        % 현재 경로점(gbest)의 가시성 여부 체크
        [~, ix] = min(abs(X(1, :) - gbest(1)));
        [~, iy] = min(abs(Y(:, 1) - gbest(2)));
        
        visibility_status = visibility_matrix(iy, ix);  % 0 = 안 보임, 1 = 보임
        visibility_values = [visibility_values; visibility_status];  % 경로점 가시성 기록


        % % 현재 지형에서의 SIR 데이터 저장
        % sir_matrix = zeros(size(Z));
        % for i = 1:size(Z, 1)
        %     for j = 1:size(Z, 2)
        %         target_pos = [X(i, j), Y(i, j), Z(i, j)];
        %         % 복수의 레이더의 경우 find_sir_multi 함수를 호출하여 모든 레이더 마다의 SIR을 계산
        %         sir_matrix(i, j) = find_sir_multi(radars, target_pos, RADAR, X, Y, Z);
        %     end
        % end
        % sir_data{end + 1} = sir_matrix;

        if abs(previous_gbest_score - gbest_score) < 1e-3
            stagnation_count = stagnation_count + 1;
        else
            stagnation_count = 0;
            search_radius = 500;
        end
        previous_gbest_score = gbest_score;
        fprintf('Current Point: (X: %.2f, Y: %.2f, Z: %.2f), Best Fitness: %.2f, Stagnation: %d\n', ...
                current_point(1), current_point(2), current_point(3), gbest_score, stagnation_count);
    end
end
% 입자 초기화 함수
function particles = initialize_particles(current_point, end_point, num_particles, radius, X, Y, Z,F)
    particles = zeros(num_particles, 3);
    direction = (end_point - current_point) / norm(end_point - current_point); % 방향 벡터 계산
    for i = 1:num_particles
        offset = randn(1, 3) * radius; % 랜덤 오프셋 생성
        offset(3) = 0; % 수평 방향으로만 랜덤 오프셋을 추가
        particles(i, :) = current_point + offset + direction * radius * rand(); % 방향성을 추가한 초기화
        particles(i, 3) = calculate_Z(particles(i, 1), particles(i, 2), X, Y, Z,F) + 30; % 고도 보정
    end
end
% 탐색 반경 내로 위치 제한
% 입자가 탐색 반경을 벗어나는 경우 반경 내 가장자리로 제한
function position = constrain_to_radius(center, position, radius, X, Y, Z,F)
    if norm(position - center) > radius
        direction = (position - center) / norm(position - center);
        position = center + direction * radius;
    end
    position(3) = calculate_Z(position(1), position(2), X, Y, Z,F) + 30; % 고도 보정
end
% x,y 좌표에서 맞는 고도 z값을 산출
function z = calculate_Z(x, y, X, Y, Z, F)
    [~, ix] = min(abs(X(1, :) - x));
    [~, iy] = min(abs(Y(:, 1) - y));
    z = Z(iy, ix);
    % 보간 적용 테스트
    % z = F(x, y);
    % if isnan(z)
    %     distances = sqrt((X(:) - x).^2 + (Y(:) - y).^2);
    %     [~, idx] = min(distances);
    %     z = Z(idx);
    % end
end
% 적합도(fitness) 계산 함수
% SIR이 최소화하는 것과 동시에 목표점까지 직선 거리를 최대한 유지하도록 설계
% 복수의 레이더 경우 find_sir_multi 적용
function fitness = calculate_fitness(radars, particle_pos, end_pos, visibility_matrix, RADAR, X, Y, Z)
    sir_value = find_sir_multi(radars, particle_pos, RADAR, X, Y, Z);
    distance_to_goal = norm(particle_pos - end_pos);
    [~, ix] = min(abs(X(1, :) - particle_pos(1)));
    [~, iy] = min(abs(Y(:, 1) - particle_pos(2)));
    % 보간 적용 테스트
    % distances = (X - particle_pos(1)).^2 + (Y - particle_pos(2)).^2;
    % [~, minIdx] = min(distances(:));
    % [iy, ix] = ind2sub(size(X), minIdx);
    visibility_bonus = 0;
    if visibility_matrix(iy, ix) == 0  % 가려진 영역이면 보상 부여
        visibility_bonus = -10;
    end

    fitness = sir_value + 0.01 * distance_to_goal + visibility_bonus;
end