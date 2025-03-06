function sir = find_sir_multi(radars, target_pos, RADAR, X, Y, Z)
    % 복수의 레이더에 대한 SIR을 계산
    % radars: 복수의 레이더 위치 (Nx3 행렬, 각 행이 레이더 위치)
    % target_pos: 목표점 위치 [x, y, z]
    % RADAR: 레이더 구조체

    num_radars = size(radars, 1);
    sir_values = -inf(num_radars, 1); % 기본값 -infinity로 초기화
    for i = 1:num_radars
        radar_pos = radars(i, :);
        sir_values(i) = find_sir_new(radar_pos, target_pos, RADAR,X,Y,Z);  
    end
    % 최대 SIR 값 반환
    % 복수의 레이더 환경에서 최대값을 가지는 SIR을 이용하기 위함
    % 단일 레이더의 경우 불필요
    % sir = max(sir_values);
    sir = max(sir_values);
end

%     num_radars = size(radars, 1);
%     sir_values = zeros(num_radars, 1);
%     for i = 1:num_radars
%         radar_pos = radars(i, :);
%         sir_values(i) = find_sir(radar_pos, target_pos, RADAR);
%     end
%     sir = max(sir_values);
% end