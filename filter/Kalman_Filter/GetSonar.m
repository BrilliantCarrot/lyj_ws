function h = GetSonar()
%GETSONAR 이 함수의 요약 설명 위치
%   자세한 설명 위치
persistent sonarAlt
persistent k firstRun


if isempty(firstRun)
    load SonarAlt
    k = 1;

    firstRun = 1;
end

h = sonarAlt(k);

k = k + 1;