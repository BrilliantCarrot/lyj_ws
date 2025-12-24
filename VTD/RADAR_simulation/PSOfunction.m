% PSO 관련 파라미터
numofParticles = 1000;
maxIterations = 100;

w = 1;
c1 = 0.75;
c2 = 0.75;
r1 = rand(1,5);
r2 = rand(1,5);

% UAV 관련 파라미터
startPos = [0,0,0];         % 초기 무인기 출발 위치
endPos = [100,100,100];     % 무인기 도착점 위치
safeDistance = 1;           % 고도 유지를 위한 안전 장치
dt = 0.1;                   % Time Stab

% 구속 조건


% PSO 초기화
particles = initializeParticles(numofParticles, startPos, endPos);
velocities = zeros(numofParticles, 3);  % 모든 입자들의 속도를 0으로 초기화
pbest = particles; % 알고리즘 시작 전에는 최적 위치를 모르므로 pbest는 입자의 위치로 설정
pbestScores = inf(numofParticles, 1);     % pbest의 점수 초기화
gbest = startPos;
gbestScore = inf;                       % gbest의 점수 초기화

%% PSO Main Loop

% PSO 실행 반복문
% 최대 반복 수에 도달할 때 까지 계산
% 전체 입자의 개수만큼 반복
% 모든 입자마다의 적합도를 계산
% 적합도 기반하여 pbest와 gbest 업데이트
for iter = 1:maxIterations
    for i = 1:numofParticles
        % 반복문 수행 내용
    end
end    

% 적합도 계산
fitness = evaluateFitness(particles(i, :), startPos, endPos, terrainHeight, safeDistance);

% pbest 업데이트
if fitness < pbestScores(i)
    pbestScores(i) = fitness;
    pbest(i, :) = particles(i, :);
end

% gbest 업데이트
if fitness < gbestScore
    gbestScore = fitness;
    gbest = particles(i, :);
end



% 입자의 속도 업데이트
% x,y,z 성분에 대해 element-wise multiplication 수행
velocities(i, :) = w * velocities(i, :) + c1 * r1 .* (pbest(i, :) - particles(i, :)) + ...
                   c2 * r2 .* (gbest - particles(i, :));
% 입자의 위치 업데이트
particles(i, :) = particles(i, :) + velocities(i, :) * dt;



% 입자 초기화 함수
function particles = initializeParticles(numofParticles, startPos, endPos)
    % 시작 위치와 마지막 위치 사이에서 입자들을 초기화
    particles = repmat(startPos, numofParticles, 1) + ...
                rand(numofParticles, 3) .* repmat(endPos - startPos, numofParticles, 1);
end

% 적합도 계산 함수
function fitness = evaluateFitness(position, startPos, endPos, terrainHeight, safeDistance, vMax, aMax)
    % Calculate SNR (example function)
    distanceToRadar = norm(position - [50, 50, 30]); % Example radar position
    snr = 1 / distanceToRadar; % Simplified SNR

    % Penalize if too close to terrain
    terrainPenalty = 0;
    if position(3) < terrainHeight(position(1), position(2)) + safeDistance
        terrainPenalty = 1000; % Large penalty for collision
    end

    % Penalize for not reaching target
    targetDistance = norm(position - endPos);

    % Combine into fitness function
    fitness = snr + terrainPenalty + targetDistance;
end