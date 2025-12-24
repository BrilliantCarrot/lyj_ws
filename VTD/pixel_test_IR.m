% IR영상 이미지에 대해 detectability를 활용하여 기하 마다의 픽셀 수 테이블을 생성

% 알고리즘대로 각각의 배경에 대해 Detectability를 제대로 산출하기 위해서
% 헬기를 촬영할 시의 캡처 화면 크기를 전부 동일하게 설정하는것이 중요함
% 이미지의 가로, 세로 사이즈를 기반으로 평균을 내기 때문

%% << Detectability >>
%% 1. 이미지 파일 하나에 대해 detectability factor를 적용 

clear;

%  헬기 및 배경을 모두 포함하는 이미지
all = imread("C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/images/winter/60/09.png");
all_temp = all;    % 픽셀 비교 임시 이미지(헬기 영역만 픽셀 값이 없는 이미지)를 위한 위의 복사본
% 지표에 대한 정보만을 포함하는 이미지
land = imread("C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/images/winter/background_before.png");

all_gray = rgb2gray(all);                   % rgb2gray 결과는 0~255의 값을 가짐
all_temp_gray = rgb2gray(all);
land_gray = rgb2gray(land);

all_double = im2double(all_gray);               % 0~1 사이의 값으로 변환
all_temp_double = im2double(all_temp_gray);
land_double = im2double(land_gray);

all_new = (all_gray/255)*100;

all_norm = 256*all_double;                 % 정규화
all_temp_norm = 256*all_temp_double;
land_norm = 256*land_double;

[rows,cols] = size(all_gray);

% 지형 배경과 헬기를 포함하는 모든 이미지의 차이를 구함
% 차이가 존재하면 헬기 영역이기에 그 영역을 0으로 만듦
heli_diff = all_norm - land_norm; 
for i = 1:rows
    for j = 1:cols
        if heli_diff(i,j) ~= 0
            all_temp_gray(i,j) = 0;     % 헬기 영역이 0으로 표현된 테이블
            % fprintf('i = %d, j = %d\n', i, j);
        end
    end
end

% 0으로 만든 이미지와 원래 이미지를 빼면 헬기에 해당하는 영역만이 나옴
heli_only = all_gray - all_temp_gray;
land_only = all_gray - heli_only;

% 0~1의 double 형태에서 구함
heli_only_double = im2double(heli_only)*100;
land_only_double = im2double(land_only)*100;

% 평균을 구하기 위해 헬기 및 지표만의 픽셀들의 개수를 구하기 위한 count 변수 설정
heli_cnt = 0;
land_cnt = 0;

% 반복문을 통해 헬기만의 픽셀 수와 지표만의 픽셀 수를 구함
for i = 1:rows
    for j = 1:cols
        if heli_only(i,j) ~= 0
            heli_cnt = heli_cnt + 1;
        end
        if land_only(i,j) ~= 0
            land_cnt = land_cnt + 1;
        end
    end
end

% 평균을 구한 값을 통하여 detectability 산출
% 0~1까지의 정규화를 수행한 값이므로 타당
% 하나의 값을 사용한다 가정하기 위하여 평균값을 이용
diff_double = abs(sum(land_only_double(:))/land_cnt - sum(heli_only_double(:))/heli_cnt);
disp('factor 적용된 결과')
disp((heli_cnt/25)*(diff_double/100))
disp(diff_double)

% 원래 코드
diff = abs(sum(land_only(:))/land_cnt - sum(heli_only(:))/heli_cnt);
disp('factor 적용된 결과')
disp((heli_cnt/25)*(diff/100))
disp(diff)

% 헬기에 해당하는 영역을 1로, 아닌영역을 0으로 이진화 시켜 헬기만 검출된게 맞는지 확인
% for i = 1:500
%     for j = 1:500
%         if heli_only(i,j) ~= 0
%             heli_only(i,j) = 1;
%         end
%     end
% end
% figure
% imshow(heli_only,[])


%% 2. 폴더 내 이미지 파일들에 대해 알고리즘을 적용

% clc;
clear;

% 경로 설정
folder_path = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/images/winter/180/%02d.png';
land_path = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/images/winter/background_after.png';

num_files = 19;  % 처리할 이미지 수
results_diff = zeros(num_files, 1);   % diff 값을 저장할 배열
results_heli_cnt = zeros(num_files, 1);  % 헬리콥터 픽셀 개수를 저장할 배열
results_land_cnt = zeros(num_files, 1);  % 배경 픽셀 개수를 저장할 배열
reults_final = zeros(num_files,1);

% 배경 이미지 로드 (변하지 않음)
land = imread(land_path);
land_gray = rgb2gray(land);
land_double = im2double(land_gray);
land_norm = 256 * land_double;
land_sum = sum(land_norm(:));

% 반복문을 통해 각 이미지를 처리
for i = 1:num_files
    % 이미지 파일 경로 생성 및 읽기
    img_path = sprintf(folder_path, i-1);
    all = imread(img_path);
    
    % 전처리 과정
    all_gray = rgb2gray(all);
    all_double = im2double(all_gray);
    all_norm = 256 * all_double;
    
    % 픽셀 값 총합 계산
    all_sum = sum(all_norm(:));
    heli_only_sum = all_sum - land_sum;
    
    % 헬기 영역 구하기
    heli_diff = all_norm - land_norm;
    all_temp_gray = all_gray;
    
    [rows, cols] = size(all_gray);
    for r = 1:rows
        for c = 1:cols
            if heli_diff(r, c) ~= 0
                all_temp_gray(r, c) = 0; % 헬기 영역을 0으로 설정
            end
        end
    end
    
    % 헬기와 배경만의 이미지 생성
    % 0부터 255까지의 범위인 데이터(0~1의 명도 값 아님)
    heli_only = all_gray - all_temp_gray;
    land_only = all_gray - heli_only;
    
    % 헬리콥터 및 배경 픽셀 수 초기화
    heli_cnt = 0;
    land_cnt = 0;
    
    % 헬리콥터와 배경 픽셀 수 계산
    for r = 1:rows
        for c = 1:cols
            if heli_only(r, c) ~= 0
                heli_cnt = heli_cnt + 1;
            end
            if land_only(r, c) ~= 0
                land_cnt = land_cnt + 1;
            end
        end
    end
    
    % 피탐성을 나타내는 diff 계산
    diff = abs(sum(land_only(:)) / land_cnt - sum(heli_only(:)) / heli_cnt);
    if diff > 100
        diff = 100-(diff-100);
    end    
    
    results_diff(i) = diff;
    results_heli_cnt(i) = heli_cnt;
    results_land_cnt(i) = land_cnt;
end

results_table_1 = table((-90:10:90)', results_diff, results_heli_cnt, results_land_cnt, ...
    'VariableNames', {'Ellevation', 'Diff', 'HelicopterPixelCount', 'LandPixelCount'});
for i = 1:19
    results_final(i) = (results_heli_cnt(i)/25)*(results_diff(i)/100);
end

for i = 1:19
    reference_pixels(i) = (results_heli_cnt(i)/25);
end

% disp(results_table_1);
% disp(reference_pixels');
% disp(round(results_final'))
disp(results_diff)
% disp(mean(diff))

%% << 함수화 >>
%% IR 피탐성 함수화

% 입력 파라미터에 대한 적용
% 입력은 수직이착륙기 관련 데이터 및 배경에 대한 정보
% 수직이착륙기의 경우: 수직이착륙기의 대략적인 온도 및 기하
% 배경의 경우: 배경 요소(봄, 여름, 가을, 겨울, 구름, 대기)


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

userTable = 'D:/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/reference_table_after.xlsx';
sheet = 1;
userTable = readmatrix(userTable, 'Sheet', sheet);

background = input('IR 카메라로 탐지되는 배경을 입력하세요: ','s');

heliTemp = input('수직이착륙기 온도를 입력하세요: ');

distance = input("카메라와 수직이착륙기 간 거리를 입력하세요: ");

% 미리 구해어진 PPM과 곱해질 detectability factor 정의
% 여름 봄 가을 구름 겨울 하늘
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

% % 수직이착륙기 기하(고각, 방위각)를 입력받음
ele = input('Enter elevation: ');
azi = input('Enter azimuth: ');
if azi <0   % 방위각이 음수일 경우 대칭성을 이용하여 양수의 범위에서 값을 찾음
    azi = abs(azi);
end  
azi = azi+2;
ele = ele+92;

% 테이블은 레퍼런스 테이블을 이용
originalPPM = userTable(ele,azi);
% PPM과 곱하여 최종적인 픽셀 수 산출
factoredPPM = originalPPM * detectabilityFactor(inputOrder);
disp('original PPM: ',originalPPM)
disp('detectability 비율이 곱해진 PPM 값: ',factoredPPM)

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

if finalPPM > minPixelCnt
    disp("목표가 식별 됨");
else
    disp("목표 식별 실패");
end

%%

% helperIRDetectability(heliTemp, azi, ele, background, distance);

%% 최종 버전 함수

% detectability 테이블을 적용
% 수직이착륙기 온도, 배경 온도, 거리를 입력받음
clear;
close all;

% detectabilityFactor 테이블 경로 설정 (각 환경에 해당하는 테이블)
% 미리 구한 detectability가 작은 순서대로 불러와야함
detectabilityTablePaths = {
    'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/detectability tables/summer.xlsx',  ... 여름
    'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/detectability tables/spring.xlsx',  ... 봄
    'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/detectability tables/autumn.xlsx',  ... 가을
    'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/detectability tables/cloud.xlsx',  ... 구름
    'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/detectability tables/winter.xlsx',  ... 겨울
    'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/detectability tables/empty_sky.xlsx',  ... 하늘
};

% detectabilityFactor에 각 환경 테이블을 동적으로 할당하기 위해 빈 셀 배열 생성
detectabilityFactor = cell(1, length(detectabilityTablePaths));

% detectabilityFactor 배열에 테이블을 동적으로 로드
for i = 1:length(detectabilityTablePaths)
    detectabilityFactor{i} = readmatrix(detectabilityTablePaths{i});
end

emptySky = 240;
winter = 270;
cloud = 275;
fall = 280;
spring = 290;
summer = 297;

% reference_table_after는 보간 이미 수행
refTable = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/reference_table_after.xlsx';
sheet = 1;
refTable = readmatrix(refTable, 'Sheet', sheet);

% 배경은 여름의 summer, 봄의 spring, 가을의 fall, 구름의 cloud, 겨울의 winter, 하늘의 empty sky를 입력
background = input('IR 카메라로 탐지되는 배경을 입력하세요: ','s');

heliTemp = input('수직이착륙기 온도를 입력하세요: ');
% 거리는 300m부터 3000m 까지를 입력
distance = input("카메라와 수직이착륙기 간 거리를 입력하세요: ");

% 미리 구해어진 PPM과 곱해질 detectability factor 정의
% 여름 봄 가을 구름 겨울 하늘
% detectabilityFactor = [0.128656, 0.277489, 0.43701, 0.457087, 0.477178, 0.8254399];

% 크기 차이 순서를 메기기 위한 배경 온도 상수로 이루어진 배열 생성
varNames = {'summer','spring','fall','cloud','winter','empty sky'};
varValues = [summer, spring, fall, cloud, winter, emptySky];

% 사용자 입력 수직이착륙기 온도와의 크기 차이를 구함
difference = abs(varValues - heliTemp);
% 수직이착륙기의 표면 온도에 따라 각 환경에서의 detectability factor를 계산
% 그 후 입력 배경에 맞는 환경 - detectability factor에 따라 PPM에 계산됨
% 크기 차이대로 순서를 새로 설정
[~, sortedIndices] = sort(difference);
sortedVars = varNames(sortedIndices);

% 크기 차이별로 순서가 정렬된 배열에서 원하는 배경을 찾음
% 크기 차이에 맞는 순서를 찾기위해 inputOder는 필요
inputOrder = find(strcmp(sortedVars, background));

% % 수직이착륙기 기하(고각, 방위각)를 입력받음
ele = input('Enter elevation: ');
azi = input('Enter azimuth: ');
if azi <0   % 방위각이 음수일 경우 대칭성을 이용하여 양수의 범위에서 값을 찾음
    azi = abs(azi);
end  
azi = azi+2;
ele = ele+92;

% 테이블은 레퍼런스 테이블을 이용
originalPPM = refTable(ele,azi);
% PPM과 곱하여 최종적인 픽셀 수 산출
selectedTable = detectabilityFactor{inputOrder};  % inputOrder에 따른 테이블 선택
detectabilityFactorValue = selectedTable(ele, azi)/100;  % 선택된 피탐성 테이블에서 고각 및 방위각 위치의 값
factoredPPM = originalPPM * detectabilityFactorValue;
disp([num2str(inputOrder),' 번째 테이블 선택됨']);
disp(['원래 PPM: ', num2str(originalPPM)]);
disp(['detectability 값: ', num2str(detectabilityFactorValue)])
disp(['detectability 비율이 곱해진 PPM 값: ', num2str(factoredPPM)]);

refPixel = 238;
minPixelCnt = 25;

% 사전에 미리 구한 이중 지수 함수의 파라미터를 이용
a = 20276.7791;
b = -0.0075;
c = 1114.5824;
d = -0.0015;

double_exp_model = @(x) a * exp(b * x) + c * exp(d * x);    % 지수 함수 피팅 모델

calculated_pixel = double_exp_model(distance);  % 비율을 구하기 위해 사용자가 입력한 거리에서 구해진 픽셀 수
pixelRatio = calculated_pixel/refPixel;     % 두 변수를 통해 비율을 계산
finalPPM = pixelRatio * factoredPPM;     % 구한 비율을 특정 기상 상황 및 특정 기하에서의 픽셀과 곱함
disp(['거리에 따른 pixel ratio: ', num2str(pixelRatio)]);
disp(['계산된 PPM 값: ', num2str(finalPPM)]);

if finalPPM > minPixelCnt
    disp("목표가 식별 됨");
else
    disp("목표 식별 실패");
end

%% 순수한 수직이착륙기 픽셀값만 존재하는 테이블과 detectabilty table을 곱함

clear;

file1 = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/reference_table_before.xlsx'; % 첫 번째 파일의 이름
data1 = readmatrix(file1);
file2 = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/보간 전 detectability table/winter.xlsx'; % 두 번째 파일의 이름
data2 = readmatrix(file2);
[size1_row, size1_col] = size(data1);
[size2_row, size2_col] = size(data2);
if size1_row ~= size2_row || size1_col ~= size2_col
    error('두 파일의 크기가 다릅니다. 동일한 크기의 데이터를 가진 파일을 사용하세요.');
end

data2_percentage = data2 / 100;
% 첫째 행과 열 제외하고 곱하기
result = data1; % 결과 저장용 배열 초기화
result(2:end, 2:end) = data1(2:end, 2:end) .* data2_percentage(2:end, 2:end);
result = result(2:end, 2:end);

output_file = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/IR PPM Table/winter.csv'; % 저장할 파일 이름
writematrix(result, output_file);

disp(['결과가 ', output_file, ' 파일로 저장되었습니다.']);

%% 구해진 csv 파일 Heatmap 형태 시각화

clear;
close all;

data = readmatrix('C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/IR PPM Table after/results_sky.csv');
data = data(2:end, 2:end); % 첫 번째 열 제거
azimuth = 0:180;    % 방위각: 0도부터 180도
elevation = -90:90; % 고각: -90도부터 90도
% azimuth = linspace(0, 180, 19);    % 방위각: adjust to 19 points
% elevation = linspace(-90, 90, 18); % 고각: adjust to 18 points

[dataRows, dataCols] = size(data);
if dataRows ~= length(elevation) || dataCols ~= length(azimuth)
    error('데이터의 크기와 방위각 또는 고각의 범위가 일치하지 않습니다.');
end

figure;
imagesc(azimuth, elevation, data);
set(gca, 'YDir', 'reverse'); % Y축 방향을 정상으로 설정
colormap(jet); % 색상 맵 설정: 파란색에서 빨간색으로
colorbar; % 컬러바 표시
xlabel('Azimuth (degree)');
ylabel('Elevation (degree)');
title('PPM - Sky Background Temperature');

%%
clear;

cl = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/IR PPM Table after/results_autumn.csv';
cl_data = readtable(cl);
cl_elevation = cl_data{2:end, 1}; % 첫 번째 열을 고각 데이터로 사용
cl_azimuth = cl_data{1, 2:end};   % 첫 번째 행을 방위각 데이터로 사용
cl_pixel_counts = cl_data{2:end, 2:end}; % 픽셀 수 데이터 가져오기

[cl_AzimuthGrid, cl_ElevationGrid] = meshgrid(cl_azimuth, cl_elevation);

figure;
contourf(cl_AzimuthGrid, cl_ElevationGrid, cl_pixel_counts, 'LineColor', 'none');
colormap(jet);
colorbar;
xlabel('Azimuth (degree)');
ylabel('Elevation (degree)');
title('Pixel Counts - Clear Environment, Reference');
grid on;
set(gca, 'YDir', 'reverse');

%% 입력 안받는 테스트 코드

clear;
close all;

detectabilityTablePaths = {
    'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/detectability tables/summer.xlsx',  ... 여름
    'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/detectability tables/spring.xlsx',  ... 봄
    'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/detectability tables/autumn.xlsx',  ... 가을
    'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/detectability tables/cloud.xlsx',  ... 구름
    'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/detectability tables/winter.xlsx',  ... 겨울
    'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/detectability tables/empty_sky.xlsx',  ... 하늘
};
detectabilityFactor = cell(1, length(detectabilityTablePaths));
for i = 1:length(detectabilityTablePaths)
    detectabilityFactor{i} = readmatrix(detectabilityTablePaths{i});
end

emptySky = 240;
winter = 270;
cloud = 275;
fall = 280;
spring = 290;
summer = 297;
refTable = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/IR/reference_table_after.xlsx';
sheet = 1;
refTable = readmatrix(refTable, 'Sheet', sheet);
varNames = {'summer','spring','fall','cloud','winter','empty sky'};
varValues = [summer, spring, fall, cloud, winter, emptySky];

% 함수에 필요한 입력 파라미터
background = 'winter';
heliTemp = 322;
distance = 3000;
ele = 0;
azi = 90;

difference = abs(varValues - heliTemp);
[~, sortedIndices] = sort(difference);
sortedVars = varNames(sortedIndices);
inputOrder = find(strcmp(sortedVars, background));

if azi <0   % 방위각이 음수일 경우 대칭성을 이용하여 양수의 범위에서 값을 찾음
    azi = abs(azi);
end  
azi = azi+2;
ele = ele+92;

originalPPM = refTable(ele,azi);
selectedTable = detectabilityFactor{inputOrder};  % inputOrder에 따른 테이블 선택
detectabilityFactorValue = selectedTable(ele, azi)/100;  % 선택된 피탐성 테이블에서 고각 및 방위각 위치의 값
factoredPPM = originalPPM * detectabilityFactorValue;
disp([num2str(inputOrder),' 번째 테이블 선택됨']);
disp(['원래 PPM: ', num2str(originalPPM)]);
disp(['detectability 값: ', num2str(detectabilityFactorValue)])
disp(['detectability 비율이 곱해진 PPM 값: ', num2str(factoredPPM)]);

refPixel = 238;
minPixelCnt = 25;

a = 20276.7791;
b = -0.0075;
c = 1114.5824;
d = -0.0015;

double_exp_model = @(x) a * exp(b * x) + c * exp(d * x);    % 지수 함수 피팅 모델

calculated_pixel = double_exp_model(distance);  % 비율을 구하기 위해 사용자가 입력한 거리에서 구해진 픽셀 수
pixelRatio = calculated_pixel/refPixel;     % 두 변수를 통해 비율을 계산
finalPPM = pixelRatio * factoredPPM;     % 구한 비율을 특정 기상 상황 및 특정 기하에서의 픽셀과 곱함
disp(['거리에 따른 pixel ratio: ', num2str(pixelRatio)]);
disp(['계산된 PPM 값: ', num2str(finalPPM)]);

if finalPPM > minPixelCnt
    disp("목표가 식별 됨");
else
    disp("목표 식별 실패");
end

%% 

clear;
clc;
close all;

path = './test/%02d.png';
num_images = 6;
temperature_data = cell(num_images, num_images);

for col = 1:num_images
    for row = 1:num_images
        img_path = sprintf(path, row-1);    % 이미지 파일 이름 생성 후 읽기
        img = imread(img_path);
        hei_gray = rgb2gray(img);
        heli_double = im2double(hei_gray);
        temp_pixel = 256 * heli_double;
        temperature_data{row, col} = temp_pixel;
    end
end

%% 

clear;
clc;
close all;

folders = 0:10:180;
num_images = 19;
temperature_data = cell(num_images, num_images);

for col = 1:num_images
    folder_path = sprintf('./IR/temp/%d/', folders(col));
    for row = 1:num_images
        img_path = sprintf('%s%02d.png', folder_path, row-1);
        img = imread(img_path);
        hei_gray = rgb2gray(img);
        heli_double = im2double(hei_gray);
        temp_pixel = 256 * heli_double;
        temperature_data{row, col} = temp_pixel;
    end
end


%%

img = imread(['C:/Users/leeyj/Unity/VTD Project/Assets/3D Haven/' ...
    'Free Fantasy Terrain Textures/Textures/2K Resolution/Temp/3DH FTT Path_001 2K.png']);
hei_gray = rgb2gray(img);

img_rgb = cat(3, hei_gray, hei_gray, hei_gray);

img_hsv = rgb2hsv(img_rgb);
V_1 = img_hsv(:,:,3);
% V 값을 0부터 255 범위로 변환 (정수형으로 변환)
V_scaled = uint8(V_1 * 255);

% disp(['최소 V 값: ', num2str(min(V_scaled(:)))]);
% disp(['최대 V 값: ', num2str(max(V_scaled(:)))]);

% 결과로 얻은 V 값을 다시 0부터 1 범위로 정규화
img_hsv(:,:,3) = double(V_scaled) / 255;
img_hsv(:,:,3) = img_hsv(:,:,3) * 2;
V_2 = img_hsv(:,:,3);

img_result = hsv2rgb(img_hsv);
% figure
% imshow(img_rgb);
% figure
% imshow(img_hsv)
figure
imshow(img_result)

%%

img = imread(['C:/Users/leeyj/Unity/VTD Project/Assets/3D Haven/' ...
    'Free Fantasy Terrain Textures/Textures/2K Resolution/Temp/3DH FTT Path_001 2K.png']);
img_hsv = rgb2hsv(img);
value = 255;
img_hsv(:,:,3) = double(value) / 255;
disp(['설정된 V 값: ', num2str(value)]);
img_result = hsv2rgb(img_hsv);
figure
imshow(img_result)

%%

% RGB 이미지 불러오기
rgbImage = imread(['C:/Users/leeyj/Unity/VTD Project/Assets/3D Haven/' ...
    'Free Fantasy Terrain Textures/Textures/2K Resolution/Temp/3DH FTT Path_001 2K.png']);

% 회색조 이미지로 변환
grayImage = rgb2gray(rgbImage);

% 회색조 이미지를 0~255 사이의 값으로 정규화
normalizedImage = double(grayImage);
normalizedImage = (normalizedImage - min(normalizedImage(:))) / (max(normalizedImage(:)) - min(normalizedImage(:))) * 255;

% 원하는 value로 설정
desiredValue = 10;  % 원하는 값을 지정하세요
normalizedImage(:) = desiredValue;

% 결과 이미지 출력
imshow(uint8(normalizedImage));

%% 
% temp{1,1} = temp_pixel;
% temperature_data{row, col} = temp_pixel;
% for i = 1:500
%     for j = 1:500
% 
%         if temp_pixel(i,j) < 53
%             temp_pixel(i,j) = nan;
%         end
%     end
% end
% T_sig = sum(sum(temp_pixel));
% max(max(temp_pixel))
% min(min(temp_pixel))

%% 

clear;
close all;

% Data and labels
values = [115.6215, 69.9341, 50.2652, 33.1567];
seasons = {'하늘', '가을', '봄', '여름'};

% Colors for each season: sky-blue, brown, yellow, blue
colors = [0.5, 0.8, 1;  % Sky color
          0.6, 0.3, 0;  % Brown
          1, 1, 0;      % Yellow
          0, 0.4, 1];   % Blue

% Create the bar plot with individual colors
figure;
b = bar(values);

% Apply specified colors and thicker edges to each bar
b.FaceColor = 'flat';
b.LineWidth = 1.5;  % Thicker border for the bars
for k = 1:length(values)
    b.CData(k, :) = colors(k, :);  % Assigns color for each bar
    b.EdgeColor = [0, 0, 0];  % Set border color to black for all bars
end

% Customize axes and labels
set(gca, 'XTickLabel', seasons);  % Set x-axis tick labels to seasons
xlabel('계절', 'FontSize', 12);
ylabel('PPM', 'FontSize', 12);
title('기하 0도, 방위각 90도, 거리 1500m, 온도 322k');

% Set y-axis label color to black
ylabel('PPM', 'Color', 'k');


%% 

% snr_values_1 = [115.6215, 69.9341, 50.2652, 33.1567];
% snr_values_2 = [19.7882, 8.5287, 3.795, 0.39721];
% snr_values_3 = [86.5563, 46.3475, 26.6488, 10.3359];
% categories = {'기하 0° 고각 90°', '기하 0° 고각 0°', '기하 30° 고각 45°'};
% figure;
% b = bar([snr_values_1; snr_values_2]', 'grouped');
% b(1).FaceColor = [0.65 0.16 0.16];  % 갈색
% b(2).FaceColor = [0.53 0.81 0.98];  % 하늘색
% b(3).FaceColor = [0.16 0.65 0.16];  % 녹색
% % bar([snr_values_1; snr_values_2]', 'grouped');
% set(gca, 'XTickLabel', categories);
% ylabel('SNR 값 (dB)');
% title('SNR 비교');
% % legend({'지표면 배경', '하늘 배경'}, 'Location', 'northwest');
% grid on;

% 케이스 1 데이터
variables = {'하늘', '겨울', '가을', '봄', '여름'};
values_case1 = [115.6215, 66.5077, 69.9341, 50.2652, 33.1567];

% 케이스 2 데이터
values_case2 = [19.7882, 10.1508, 8.5287, 3.7950, 0.39721];

% 케이스 3 데이터
values_case3 = [86.5563, 50.5515, 46.3475, 26.6488, 10.3359];

% 케이스 1 그래프
figure;
bar(values_case1);
set(gca, 'XTickLabel', variables, 'FontSize', 12);
xlabel('변수', 'FontSize', 14);
ylabel('값', 'FontSize', 14);
title('케이스 1: 기하 고각 0°, 방위각 90°', 'FontSize', 16);
grid on;

% 케이스 2 그래프
figure;
bar(values_case2);
set(gca, 'XTickLabel', variables, 'FontSize', 12);
xlabel('변수', 'FontSize', 14);
ylabel('값', 'FontSize', 14);
title('케이스 2: 기하 고각 0°, 방위각 0°', 'FontSize', 16);
grid on;

% 케이스 3 그래프
figure;
bar(values_case3);
set(gca, 'XTickLabel', variables, 'FontSize', 12);
xlabel('변수', 'FontSize', 14);
ylabel('값', 'FontSize', 14);
title('케이스 3: 기하 고각 30°, 방위각 45°', 'FontSize', 16);
grid on;
