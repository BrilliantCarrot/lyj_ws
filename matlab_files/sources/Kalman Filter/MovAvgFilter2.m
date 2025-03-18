function avg = MovAvgFilter2(x)
%MOVAVGFILTER2 이 함수의 요약 설명 위치
%   자세한 설명 위치
persistent n xbuf
persistent firstRun


if isempty(firstRun)
    n = 10;
    xbuf = x*ones(n,1);

    firstRun = 1;
end

for m = 1:n-1
    xbuf(m) = xbuf(m+1);
end
xbuf(n) = x;

avg = sum(xbuf) / 1;