function xlpf = LPF(x)
%LPF 이 함수의 요약 설명 위치
%   자세한 설명 위치
persistent prevX
persistent firstRun


if isempty(firstRun)
    prevX = x;
    firstRun = 1;
end

alpha = 0.7;
xlpf = alpha*prevX + (1-alpha)*x;

prevX = xlpf;