%% 1개의 이미지에만 대해 픽셀 검출흑백)
% 코드 경로: C:/Users/leeyj/lab_ws/data/vtd/EO

clear;
% clc;
close all;

% 캡처한 수직이착륙기 파일
% img_forest = imread('../data/검정도색 여름 숲 주간 우측.png');
% img_river = imread('../data/검정도색 여름 강 주간 우측.png');
% img_snow = imread('../data/검정도색 겨울 주간 우측.png');
% img_brown = imread('../data/검정도색 여름 황토 주간 우측.png');
% img_reference = imread("../data/레퍼런스 이미지.png");
% img_bwarea = imread("../data/bwarea.png");

% img = imread("./fog/land/temp/01.png");
img = imread("C:/Users/leeyj/lab_ws/data/vtd/EO/distance/new_0/01.png");
% figure
% imshow(img);
% title("원본 이미지");

upper_bound = [50, 50, 50];
lower_bound = [0, 0, 0];

% 경계 값 설정을 통해 특정 색상의 헬기 픽셀 검출
% lower_bound = [70, 120, 60];
% upper_bound = [140, 50, 70];
% 숲 + 안개(+비)
% upper_bound = [160, 155, 145];
% lower_bound = [155, 150, 140];
% 숲 + 비
% upper_bound = [142, 125, 82];
% lower_bound = [127, 110, 75];
% 숲 + 안개(+굵은 비)
% upper_bound = [160, 150, 140];
% lower_bound = [150, 145, 130];
% 황무지 + 안개(+굵은 비)
% upper_bound = [139, 140, 140];
% lower_bound = [136, 135, 135];
% 황무지 + 안개(+비)
% upper_bound = [117, 120, 117];
% lower_bound = [111, 117, 111];
% 구름 + 안개(+비)
% upper_bound = [165, 165, 165];
% lower_bound = [164, 164, 163];
% 눈 + 안개
% upper_bound = [172, 171, 167];
% lower_bound = [170, 169, 165];

binaryImg = img(:,:,1) >= lower_bound(1) & img(:,:,1) <= upper_bound(1) & ...
            img(:,:,2) >= lower_bound(2) & img(:,:,2) <= upper_bound(2) & ...
            img(:,:,3) >= lower_bound(3) & img(:,:,3) <= upper_bound(3);

% figure
% imshow(binaryImg);
% title("이진화 적용 이미지");

binaryImg = bwareaopen(binaryImg,5);    % 특정 픽셀보다 작은 인식 결과는 제외

% figure
% imshow(binaryImg);
% title("bwareaopen 적용");

% 이진화와 불필요 픽셀을을 모두 제거한 결과 픽셀들의 sum을 구함
helicopterPixels = sum(binaryImg(:));
% unity에서 표현되는 픽셀들의 개수
% Unity의 픽셀 수 대로 세기위해 25로 나눔(유니티 한 픽셀의 크기는 캡처된 사진에서 5X5 픽셀)
unityPixelsNum = helicopterPixels/25;   


% 경계 영역 시각화용 테두리 생성
contours = bwperim(binaryImg);
% figure
% imshow(contours);
% title("경계영역만 표현")

% 원 이미지에 테두리를 그려 검출된 헬기 픽셀을 시각화
% figure
% imshow(img);
% hold on;
% visboundaries(contours, 'Color', 'r');
% title("픽셀 구분 최종 결과");
% hold off;

fprintf('헬기로 인식된 픽셀의 총 갯수: %d\n', unityPixelsNum);

%% 반복문으로 이미지 파일들에 대해 개별 픽셀 검출 수행

clear;
clc;
close all;

% 경로 설정
% 파이썬 opencv를 통해 전처리된(마스킹된) 이미지에 대하여 픽셀 검출
% 파이썬 실행 후 이 매트랩 파일을 실행
path = './clear/final/180/%02d.png';      % vscode를 통해 미리 생성된 마스킹된 결과 이미지가 있는 경로
num_files = 19;

% 결과를 저장할 테이블 초기화
% results = table('Size', [num_files, 2], 'VariableTypes', {'string', 'double'}, 'VariableNames', {'ImageName', 'UnityPixelsNum'});

% 결과를 저장할 테이블 초기화
results = table('Size', [num_files, 1], 'VariableTypes', {'double'}, 'VariableNames', {'UnityPixelsNum'});

for i = 1:num_files
    img_name = sprintf(path, i-1);
    img = imread(img_name);

    % forest
    upper_bound = [80, 70, 60];
    lower_bound = [0, 0, 0];
    

    binaryImg = img(:,:,1) >= lower_bound(1) & img(:,:,1) <= upper_bound(1) & ...
                img(:,:,2) >= lower_bound(2) & img(:,:,2) <= upper_bound(2) & ...
                img(:,:,3) >= lower_bound(3) & img(:,:,3) <= upper_bound(3);

    binaryImg = bwareaopen(binaryImg, 250);    % 특정 픽셀보다 작은 인식 결과는 제외

    % 이진화와 불필요 픽셀을 모두 제거한 결과 픽셀들의 sum을 구함
    helicopterPixels = sum(binaryImg(:));
    % Unity에서 표현되는 픽셀들의 개수
    % Unity의 픽셀 수 대로 세기위해 25로 나눔(유니티 한 픽셀의 크기는 캡처된 사진에서 5X5 픽셀)
    unityPixelsNum = round(helicopterPixels / 25);     
    
    % 결과를 테이블에 저장
    % results.ImageName(i) = {img_name};    % 열이 2개 테이블인 경우
    results.UnityPixelsNum(i) = unityPixelsNum;
    

    % 결과 시각화
    % contours = bwperim(binaryImg);
    % figure
    % imshow(img);
    % hold on;
    % visboundaries(contours, 'Color', 'r');
    % title(sprintf('픽셀 구분 최종 결과 - 파일 %02d', i-1));
    % pause(0.05)
    % hold off;
end

% 결과 출력
disp(results);

% 결과를 엑셀이나 csv 파일로 저장
writetable(results, 'Results.csv');

close all;

% sky background upper bound 110 110 110

% 안개 
% upper_bound = [132, 132, 130];
% lower_bound = [70, 70, 70];
% RGB 값 기록
% 지면 125 125 123
% 구름 150 148 144, 152 149 143, 142 144 141

% 구름
% upper_bound = [152, 152, 152];
% lower_bound = [70, 70, 70];

% 눈
% upper_bound = [168, 162, 159];
% lower_bound = [70, 70, 70];

% heavy_rain
% upper_bound = [110, 110, 100];
% lower_bound = [70, 70, 70];

% forest
% upper_bound = [135, 125, 85];
% lower_bound = [0, 0, 0];

%% 히트맵

clear;
clc;
close all;

data = readtable('Pixels.xlsx');
elevation = data{:, 1};
azimuth = data{1,:};
pixel_counts = data{:,:};
% pixel_counts(isnan(pixel_counts)) = 0;

figure;
imagesc(azimuth, elevation, pixel_counts);
colormap('jet');
colorbar;
xlabel('Azimuth (degrees)');
ylabel('Elevation (degrees)');
title('Helicopter Pixel Count Visualization');
set(gca, 'YDir', 'normal');

%% 등고선

% cl = clear;
% ml = moderate rain
% hl = heavy rain

clear;
clc;
close all;

cl = './clear/clear sky.xlsx';
cl_data = readtable(cl);
cl_elevation = cl_data{2:end, 1};
cl_azimuth = cl_data{1,2:end};
cl_pixel_counts = cl_data{2:end, 2:end};
cl_pixel_counts_percentage = (cl_pixel_counts/max(cl_pixel_counts(:)))*100;
[cl_AzimuthGrid, cl_ElevationGrid] = meshgrid(cl_azimuth, cl_elevation);

ml = './moderate_rain/land/지면 배경, 우천시.xlsx';
ml_data = readtable(ml);
ml_elevation = ml_data{2:end, 1};
ml_azimuth = ml_data{1,2:end};
ml_pixel_counts = ml_data{2:end, 2:end};
ml_pixel_counts_percentage = (ml_pixel_counts/max(cl_pixel_counts(:)))*100;
[ml_AzimuthGrid, ml_ElevationGrid] = meshgrid(ml_azimuth, ml_elevation);

hl = './heavy_rain/sky/폭우 환경, 하늘 배경.xlsx';
hl_data = readtable(hl);
hl_elevation = hl_data{2:end, 1};
hl_azimuth = hl_data{1,2:end};
hl_pixel_counts = hl_data{2:end, 2:end};
hl_pixel_counts_percentage = (hl_pixel_counts/max(cl_pixel_counts(:)))*100;
[hl_AzimuthGrid, hl_ElevationGrid] = meshgrid(hl_azimuth, hl_elevation);

fog = './fog/land/지면 배경, 안개.xlsx';
fog_data = readtable(fog);
fog_elevation = fog_data{2:end, 1};
fog_azimuth = fog_data{1,2:end};
fog_pixel_counts = fog_data{2:end, 2:end};
fog_pixel_counts_percentage = (fog_pixel_counts/max(cl_pixel_counts(:)))*100;
[fog_AzimuthGrid, fog_ElevationGrid] = meshgrid(fog_azimuth, fog_elevation);

cloud = './cloud/sky/하늘 배경, 구름.xlsx';
cloud_data = readtable(cloud);
cloud_elevation = cloud_data{2:end, 1};
cloud_azimuth = cloud_data{1,2:end};
cloud_pixel_counts = cloud_data{2:end, 2:end};
cloud_pixel_counts_percentage = (cloud_pixel_counts/max(cl_pixel_counts(:)))*100;
[cloud_AzimuthGrid, cloud_ElevationGrid] = meshgrid(cloud_azimuth, cloud_elevation);

snow = './snow/설원 배경, 강설.xlsx';
snow_data = readtable(snow);
snow_elevation = snow_data{2:end, 1};
snow_azimuth = snow_data{1,2:end};
snow_pixel_counts = snow_data{2:end, 2:end};
snow_pixel_counts_percentage = (snow_pixel_counts/max(cl_pixel_counts(:)))*100;
[snow_AzimuthGrid, snow_ElevationGrid] = meshgrid(snow_azimuth, snow_elevation);

forest = './forest/숲 배경.xlsx';
forest_data = readtable(forest);
forest_elevation = forest_data{2:end, 1};
forest_azimuth = forest_data{1,2:end};
forest_pixel_counts = forest_data{2:end, 2:end};
forest_pixel_counts_percentage = (forest_pixel_counts/max(cl_pixel_counts(:)))*100;
[forest_AzimuthGrid, forest_ElevationGrid] = meshgrid(forest_azimuth, forest_elevation);

% clear = './clear/clear sky.xlsx';
% clear_data = readtable(clear);
% elevation = clear_data{2:end, 1};
% azimuth = clear_data{1,2:end};
% clear_pixel_counts = clear_data{2:end, 2:end};
% clear_pixel_counts_percentage = (clear_pixel_counts/max(clear_pixel_counts(:)))*100;

% AzimuthGrid = flipud(AzimuthGrid);
% ElevationGrid = flipud(ElevationGrid);
% pixel_counts = flipud(pixel_counts);
% pixel_counts_percentage = flipud(pixel_counts_percentage);

% 픽셀 비율에 대해 등고선 Plot

% 기준 픽셀
figure;
contourf(cl_AzimuthGrid, cl_ElevationGrid, cl_pixel_counts_percentage, 'LineColor', 'none');
colormap(jet);
colorbar;
xlabel('Azimuth (degree)');
ylabel('Elevation (degree)');
title('Percentage of Pixels - Clear Env, Reference');
grid on;
set(gca, 'YDir', 'reverse');

% 현재 컬러맵 범위 저장
% caxis_range = caxis;

% moderate rain 픽셀
figure;
contourf(ml_AzimuthGrid, ml_ElevationGrid, ml_pixel_counts_percentage, 'LineColor', 'none');
colormap(jet);
colorbar;
xlabel('Azimuth (degree)');
ylabel('Elevation (degree)');
title('Percentage of Pixels - Moderate Rain Env');
grid on;
set(gca, 'YDir', 'reverse');

% 컬러맵 범위를 첫 번째 그래프에 맞춤
% caxis(caxis_range);
% Colorbar의 범위를 원래 데이터 범위로 설정
% c.Limits = [0, max(clear_pixel_counts_percentage(:))];

% heavy rain 픽셀
figure;
contourf(hl_AzimuthGrid, hl_ElevationGrid, hl_pixel_counts_percentage, 'LineColor', 'none');
colormap(jet);
colorbar;
xlabel('Azimuth (degree)');
ylabel('Elevation (degree)');
title('Percentage of Pixels - Heavy Rain Env');
grid on;
set(gca, 'YDir', 'reverse');
% caxis(caxis_range);

% fog 픽셀
figure;
contourf(fog_AzimuthGrid, fog_ElevationGrid, fog_pixel_counts_percentage, 'LineColor', 'none');
colormap(jet);
colorbar;
xlabel('Azimuth (degree)');
ylabel('Elevation (degree)');
title('Percentage of Pixels - Fog Env');
grid on;
set(gca, 'YDir', 'reverse');
% caxis(caxis_range);

% cloud 픽셀
figure;
contourf(cloud_AzimuthGrid, cloud_ElevationGrid, cloud_pixel_counts_percentage, 'LineColor', 'none');
colormap(jet);
colorbar;
xlabel('Azimuth (degree)');
ylabel('Elevation (degree)');
title('Percentage of Pixels - Cloud Env');
grid on;
set(gca, 'YDir', 'reverse');

% snow 픽셀
figure;
contourf(snow_AzimuthGrid, snow_ElevationGrid, snow_pixel_counts_percentage, 'LineColor', 'none');
colormap(jet);
colorbar;
xlabel('Azimuth (degree)');
ylabel('Elevation (degree)');
title('Percentage of Pixels - Snow Env');
grid on;
set(gca, 'YDir', 'reverse');

% forest 픽셀
figure;
contourf(forest_AzimuthGrid, forest_ElevationGrid, forest_pixel_counts_percentage, 'LineColor', 'none');
colormap(jet);
colorbar;
xlabel('Azimuth (degree)');
ylabel('Elevation (degree)');
title('Percentage of Pixels - Forest Env');
grid on;
set(gca, 'YDir', 'reverse');

% 픽셀 개수에 대해 등고선 Plot
% figure;
% contourf(AzimuthGrid, ElevationGrid, pixel_counts, 'LineColor', 'none');
% colormap(jet);
% colorbar;
% xlabel('Azimuth (degrees)');
% ylabel('Elevation (degrees)');
% title('Pixel Counts Contour ');
% grid on;
% set(gca, 'YDir', 'reverse');


%% 최대, 최소, 평균 값 찾기

clear;
clc;
close all;

filename = 'C:/Users/leeyj/lab_ws/data/vtd/EO/Table_of_Pixels.csv';
data = readtable(filename);

elevation = data{2:end, 1};
azimuth = data{1,2:end};
pixel_counts = data{2:end, 2:end};

pixel_counts_percentage = (pixel_counts/247)*100;

[AzimuthGrid, ElevationGrid] = meshgrid(azimuth, elevation);

figure;
contourf(AzimuthGrid, ElevationGrid, pixel_counts, 'LineColor', 'none');
colormap(jet);
colorbar;
xlabel('Azimuth (degrees)');
ylabel('Elevation (degrees)');
title('Pixel Counts Contour');
grid on;
set(gca, 'YDir', 'reverse');

figure;
contourf(AzimuthGrid, ElevationGrid, pixel_counts_percentage, 'LineColor', 'none');
colormap(jet);
colorbar;
xlabel('Azimuth (degrees)');
ylabel('Elevation (degrees)');
title('Percentage of Pixels');
grid on;
set(gca, 'YDir', 'reverse');

max_value = max(pixel_counts(:));
min_value = min(pixel_counts(:));
mean_value = sum(pixel_counts(:))/361;

% 평균값과 가장 가까운 값 찾기
[~, closest_index] = min(abs(pixel_counts(:) - mean_value));
closest_value = pixel_counts(closest_index);

fprintf('최대값: %.2f\n', max_value);
fprintf('최소값: %.2f\n', min_value);
fprintf('평균값: %.2f\n', mean_value);
fprintf('평균값과 가장 가까운 값: %.2f\n', closest_value);


%% 거리 증감에 따른 픽셀 개수 변화 시각화

clc;
clear;
close all;

% 0 degree azimuth angle
data1 = [15717, 6406, 2817, 1576, 996, 676, 494, 378, 293, 195];

% 135 degree azimuth angle
% data2 = [12677, 4997, 2182, 1208, 773, 528, 365, 284, 173, 146];

x = 1:length(data1);
x_labels = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100];
figure;
hold on;
plot(x, data1, '-o', 'LineWidth', 2, 'DisplayName', 'Data Set 1');
% plot(x, data2, '-o', 'LineWidth', 2, 'DisplayName', 'Data Set 2');
hold off;
xlabel('Distance [m]');
ylabel('Pixel Counts');
title('Pixel Counts per Distances');
legend("0 dgree azimuth","135 degree azimuth");
xticks(x);                  % x축 위치 설정
xticklabels(x_labels);      % x축 레이블 설정
grid on;

%% 거리 기반 지수함수 피팅



%% 새로 뽑은 데이터로, 비선형 회귀 함수
% x와 y 데이터 정의
% x = (100:100:1500);
% y = [9858, 2392, 1037, 580, 362, 250, 186, 138, 108, 84, 69, 58, 52, 45, 39];

x = (200:100:1000);
y = [2392, 1037, 580, 362, 250, 186, 138, 108, 84];

% 지수함수 모델을 정의하는 익명함수 (a * exp(b * x))
exp_model = @(a, b, x) a * exp(b * x);

% 비선형 모델 피팅을 위한 시작 추정값
initial_guess = [y(1), -0.001];  % a의 초기값은 첫 번째 y 값, b는 작은 음수 값으로 설정

% 피팅을 위한 비선형 회귀 함수 사용
fit_func = @(params) sum((exp_model(params(1), params(2), x) - y).^2);  % 최소 제곱법

% fminsearch로 최적의 a와 b 찾기
optimal_params = fminsearch(fit_func, initial_guess);

% 최적의 a, b 값을 출력
a_opt = optimal_params(1);
b_opt = optimal_params(2);
fprintf('a = %.4f, b = %.4f\n', a_opt, b_opt);

% 피팅된 지수함수로 y 값 예측
y_fit = exp_model(a_opt, b_opt, x);

% 원래 데이터와 피팅된 데이터를 시각화
figure;
plot(x, y, 'bo-', 'LineWidth', 2); % 원래 데이터
hold on;
plot(x, y_fit, 'r--', 'LineWidth', 2); % 피팅된 지수함수
xlabel('x');
ylabel('y');
title('Exponential Fit to Data');
legend('Original Data', 'Fitted Exponential Curve');
grid on;


%% 새로 뽑은 데이터로, lsqcurvefit

% 데이터 설정
x = 100:100:1500; % x값
y = [9858, 2392, 1037, 580, 362, 250, 186, 138, 108, 84, 69, 58, 52, 45, 39]; % 주어진 y값

% 지수 함수 정의
exp_fun = @(c, x) c(1)*exp(c(2)*x);

% 초기 추정값 (a와 b 값)
initial_guess = [10000, -0.001];

% 피팅 수행
c_fit = lsqcurvefit(exp_fun, initial_guess, x, y);

% 피팅 결과 출력
disp('Fitted Parameters:');
disp(c_fit);

% 피팅된 모델 그리기
x_fit = linspace(min(x), max(x), 100);
y_fit = exp_fun(c_fit, x_fit);

figure;
plot(x, y, 'o', x_fit, y_fit, '-');
title('Exponential Fit using lsqcurvefit');
xlabel('x');
ylabel('y');
legend('Data', 'Exponential Fit');
grid on;

%% 이전 데이터로

x = (200:100:1100);
y = [4997, 2182, 1208, 773, 528, 365, 284, 216, 173, 146];
% 지수함수 모델을 정의하는 익명함수 (a * exp(b * x))
exp_model = @(a, b, x) a * exp(b * x);
% 비선형 모델 피팅을 위한 시작 추정값
initial_guess = [y(1), -0.001];  % a의 초기값은 첫 번째 y 값, b는 작은 음수 값으로 설정

% 피팅을 위한 비선형 회귀 함수 사용
fit_func = @(params) sum((exp_model(params(1), params(2), x) - y).^2);  % 최소 제곱법

% fminsearch로 최적의 a와 b 찾기
optimal_params = fminsearch(fit_func, initial_guess);

% 최적의 a, b 값을 출력
a_opt = optimal_params(1);
b_opt = optimal_params(2);
fprintf('a = %.4f, b = %.4f\n', a_opt, b_opt);

% 피팅된 지수함수로 y 값 예측
y_fit = exp_model(a_opt, b_opt, x);

% 원래 데이터와 피팅된 데이터를 시각화
figure;
plot(x, y, 'bo-', 'LineWidth', 2); % 원래 데이터
hold on;
plot(x, y_fit, 'r--', 'LineWidth', 2); % 피팅된 지수함수
xlabel('x');
ylabel('y');
title('Exponential Fit to Data');
legend('Original Data', 'Fitted Exponential Curve');
grid on;

% [height, width, numChannels] = size(img);
% fprintf('Image size: %d x %d x %d\n', height, width, numChannels);

% 히스토그램 평활화
% adjustedImg = img;
% adjustedImg(:,:,1) = imadjust(img(:,:,1));
% adjustedImg(:,:,2) = imadjust(img(:,:,2));
% adjustedImg(:,:,3) = imadjust(img(:,:,3));
% figure;
% subplot(1,2,1);
% imshow(img);
% title("Original Image");
% subplot(1,2,2);
% imshow(adjustedImg);
% title("히스토그램 평활화 수행")

% 히스토그램 분포도를 시각화
% figure;
% subplot(3, 1, 1);
% imhist(adjustedImg(:,:,1));
% title('Adjusted Red Channel Histogram');
% subplot(3, 1, 2);
% imhist(adjustedImg(:,:,2));
% title('Adjusted Green Channel Histogram');
% subplot(3, 1, 3);
% imhist(adjustedImg(:,:,3));
% title('Adjusted Blue Channel Histogram');

% 헬기의 위치 정보를 기반으로 객체를 선택합니다.
% (헬기의 위치 정보를 알고 있다면 사용합니다.)
% 예를 들어, 헬기가 이미지 중앙에 위치한다고 가정할 때

% center = round(size(binaryImg) / 2);
% search_radius = 100; % 검색 반경 설정
% mask = false(size(binaryImg));
% mask(center(1)-search_radius:center(1)+search_radius, ...
%      center(2)-search_radius:center(2)+search_radius) = true;
% binaryImg = binaryImg & mask;

