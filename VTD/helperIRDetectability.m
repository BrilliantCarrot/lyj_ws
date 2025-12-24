function helperIRDetectability(heliTemp,azi,ele,background,distance) 

% 수직 이착륙기의 기하, 표면 온도, 배경 온도를 파라미터로 받아
% 그 때의 픽셀 수를 Detectability Factor를 기반으로 하여 산출
% DORI 기준에 의해 픽셀 수가 25 ppm 이상일 시 검출 되며 그 미만일 시 검출이 안됨

% 변수 설명
% heliTemp: 수직이착륙기의 표면 온도(켈빈 온도)
% azi 및 ele: 수직이착륙기의 기하
% background: 수직이착륙기 뒤의 배경(대기, 구름, 겨울, 가을, 봄, 여름 6가지)
% detectabilityFactor: 수직이착륙기와 배경의 명도 차이에 의해 계산되어 픽셀에 곱해질 비율이 저장된 배열
% PPM: 기하에 따라 테이블에서 선정 된 base 픽셀 값, detectabiliy factor에 곱해짐

% 함수의 사용 예시
% 수직이착륙기의 기하가 ##,##로 주어졌으며 표면 온도가 ##K, 그리고 수직이착륙기의 배경이 #일때 
% 그 상황에서 수직이착륙기가 보이는가 안 보이는가를 결과로 출력

% Detectability Factor를 수직이착륙기 표면 온도에 따라 설정하기 위한 값을 미리 선언
% 사용자의 입력 background에 따라 계산된 detectability factor의 결과 픽셀 수를 산출

% 미리 구해진 켈빈 온도로 표현된 배경 온도 상수
emptySky = 240;
winter = 270;
cloud = 275;
fall = 280;
spring = 290;
summer = 297;

background = input('IR 카메라로 탐지되는 배경을 입력하세요: ');

% 미리 구해어진 PPM과 곱해질 detectability factor 정의
detectabilityFactor = [0.128656, 0.277489, 0.43701, 0.457087, 0.477178, 0.8254399];

% 크기 차이 순서를 메기기 위한 배경 온도 상수로 이루어진 배열 생성
varNames = {'empty sky','winter','cloud','fall','spring','summer'};
varValues = [emptySky, winter, cloud, fall, spring, summer];

% 사용자 입력 수직이착륙기 온도와의 크기 차이를 구함
difference = abs(varValues - heliTemp);

% 수직이착륙기의 표면 온도에 따라 각 환경에서의 detectability factor를 계산
% 그 후 입력 배경에 맞는 환경 - detectability factor에 따라 PPM에 계산됨

% 크기 차이대로 순서를 새로 설정
[~, sortedIndices] = sort(difference);
sortedVars = varNames(sortedIndices);

% 크기 차이별로 순서가 정렬된 배열에서 원하는 배경을 찾음
inputOrder = find(strcmp(sortedVars, background));

ele = input('Enter elevation: ');
azi = input('Enter azimuth: ');

azi = azi+2;
ele = ele+92;

% 테이블은 레퍼런스 테이블을 이용
PPM = userTable(ele,azi);
% PPM과 곱하여 최종적인 픽셀 수 산출
PPM = PPM * detectabilityFactor(inputOrder);
disp(PPM);

refPixel = 230;
minPixelCnt = 25;

% 사전에 미리 구한 이중 지수 함수의 파라미터를 이용
a = 20276.7791;
b = -0.0075;
c = 1114.5824;
d = -0.0015;

double_exp_model = @(x) a * exp(b * x) + c * exp(d * x);    % 지수 함수 피팅 모델

calculated_pixel = double_exp_model(distance);  % 비율을 구하기 위해 사용자가 입력한 거리에서 구해진 픽셀 수
pixelRatio = calculated_pixel/refPixel;     % 두 변수를 통해 비율을 계산
finalPPM = pixelRatio * originalPixel;     % 구한 비율을 특정 기상 상황 및 특정 기하에서의 픽셀과 곱함

disp(['계산된 PPM 값: ', num2str(finalPPM)]);

if finalPixelcnt > minPixelCnt
    disp("목표가 식별 됨");
end