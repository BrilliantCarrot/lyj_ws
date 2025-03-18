function [volt, Px, K] = SimpleKalman2(z)
%SIMPLEKALMAN 이 함수의 요약 설명 위치
%   자세한 설명 위치
persistent A H Q R
persistent x P
persistent firstRun


if isempty(firstRun)
    A = 1;
    H = 1;

    Q = 0;
    R = 4;

    x = 14;
    P = 6;

    firstRun = 1;
end


xp = A*x;                       % 추정값 예측   
Pp = A*P*A' + Q;                % 오차 공분산 예측

K = Pp*H'/(H*Pp*H' + R);        % 칼만 이득 계산

x = xp + K*(z-H*xp);            % 추정값 계산
P = Pp - K*H*Pp;                % 오차 공분산 계산


volt = x;                       % 추정값 반환
Px = P;                         