function [optimal_path, sir_data] = PSO_SIR_Optimization(radar_pos, start_pos, end_pos, X, Y, Z, RADAR)
    % PSO 파라미터 정의
    num_particles = 500;   % 입자 수 줄임 (더 작은 탐색 영역)
    max_iter = 50;         % 반복 수
    w = 0.7;               % 관성 계수
    c1 = 1.5;              % 개인 가속 계수
    c2 = 1.5;              % 전역 가속 계수

    % 탐색 반경 설정
    search_radius = 500;   % 초기 탐색 반경 (m)
    min_distance = 30;     % 목표점에 도달했다고 간주하는 거리

    % 결과 경로 초기화
    optimal_path = start_pos;
    current_point = start_pos;
    sir_data = {};

    while norm(current_point - end_pos) > min_distance
        % 입자의 위치와 속도 초기화
        particles = initialize_particles(current_point, end_pos, num_particles, search_radius, X, Y, Z);
        velocities = zeros(size(particles));

        % 초기 개인 최적값 및 전역 최적값 설정
        pbest = particles;
        pbest_scores = arrayfun(@(i) calculate_fitness(radar_pos, particles(i, :), end_pos, RADAR), 1:num_particles)';
        [gbest_score, gbest_idx] = min(pbest_scores);
        gbest = pbest(gbest_idx, :);

        % PSO 반복문
        for iter = 1:max_iter
            for i = 1:num_particles
                % 현재 입자의 SIR 계산
                current_score = calculate_fitness(radar_pos, particles(i, :), end_pos, RADAR);

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
                particles(i, :) = constrain_to_radius(current_point, particles(i, :), search_radius, X, Y, Z);
            end
        end

        % 다음 경로 점 업데이트
        current_point = gbest;
        optimal_path = [optimal_path; current_point];

        % 현재 지형에서의 SIR 데이터 저장
        sir_matrix = zeros(size(Z));
        for i = 1:size(Z, 1)
            for j = 1:size(Z, 2)
                target_pos = [X(i, j), Y(i, j), Z(i, j)];
                sir_matrix(i, j) = find_sir(radar_pos, target_pos, RADAR);
            end
        end
        sir_data{end + 1} = sir_matrix;

        fprintf('Current Point: (X: %.2f, Y: %.2f, Z: %.2f), Best Fitness: %.2f\n', ...
                current_point(1), current_point(2), current_point(3), gbest_score);
    end
end

% 입자 초기화 함수
function particles = initialize_particles(current_point, end_point, num_particles, radius, X, Y, Z)
    particles = zeros(num_particles, 3);
    direction = (end_point - current_point) / norm(end_point - current_point); % 방향 벡터 계산

    for i = 1:num_particles
        offset = randn(1, 3) * radius; % 랜덤 오프셋 생성
        offset(3) = 0; % 수평 방향으로만 랜덤 오프셋을 추가
        particles(i, :) = current_point + offset + direction * radius * rand(); % 방향성을 추가한 초기화
        particles(i, 3) = calculate_Z(particles(i, 1), particles(i, 2), X, Y, Z) + 100; % 고도 보정
    end
end


% 탐색 반경 내로 위치 제한
% 입자가 탐색 반경을 벗어나는 경우 반경 내 가장자리로 제한
function position = constrain_to_radius(center, position, radius, X, Y, Z)
    if norm(position - center) > radius
        direction = (position - center) / norm(position - center);
        position = center + direction * radius;
    end
    position(3) = calculate_Z(position(1), position(2), X, Y, Z) + 100; % 고도 보정
end

% x,y 좌표에서 맞는 고도 z값을 산출
function z = calculate_Z(x, y, X, Y, Z)
    [~, ix] = min(abs(X(1, :) - x));
    [~, iy] = min(abs(Y(:, 1) - y));
    z = Z(iy, ix);
end

% 적합도(fitness) 계산 함수
% SIR이 최소화하는 것과 동시에 목표점까지 직선 거리를 최대한 유지하도록 설계
function fitness = calculate_fitness(radar_pos, particle_pos, end_pos, RADAR)
    % SIR과 거리의 가중합을 통해 적합도 산출
    % SIR 값 계산
    sir_value = find_sir(radar_pos, particle_pos, RADAR);
    % 목표점까지의 거리 계산
    distance_to_goal = norm(particle_pos - end_pos);
    % SIR 가중치를 곱해 SIR이 낮더라도 목표점까지 가까워지도록 유도
    % 최적 경로가 SIR뿐만 아니라 목표점까지의 이동 효율성도 고려하여 탐색
    fitness = sir_value + 0.01 * distance_to_goal;
end