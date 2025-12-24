function z = GetPos()
%GETPOS 이 함수의 요약 설명 위치
%   자세한 설명 위치
persistent Velp Posp


if isempty(Posp)
    Posp = 0;
    Velp = 80;
end

dt = 0.1;

w = 0 + 10*randn;
v = 0 + 10*randn;

z = Posp + Velp*dt + v;

Posp = z - v;
Velp = 80 + w;