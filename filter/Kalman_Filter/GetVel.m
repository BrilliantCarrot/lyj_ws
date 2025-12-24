function z = GetVel()
%GETPOS 이 함수의 요약 설명 위치
%   자세한 설명 위치
persistent Velp Posp


if isempty(Posp)
    Posp = 0;
    Velp = 80;
end

dt = 0.1;

v = 0 + 10*randn;

Posp = Posp + Velp*dt;
Velp = 80 + v;

z = Velp;