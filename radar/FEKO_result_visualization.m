close all
clc;
clear;

% CSV 파일 경로 지정
filePath = 'C:/Users/leeyj/OneDrive - 인하대학교/School/Matlab_Files/RADAR/data/rcs_table/Tomahawk_PO_3D_1_300MHz_FFE_Mat.csv';

% CSV 파일을 행렬 형태로 불러오기
data = csvread(filePath);

% 고각 (Theta) 및 방위각 (Phi) 범위 설정
theta = 0:180; % 0도에서 180도까지 1도 간격
phi = 0:360;   % 0도에서 360도까지 1도 간격

% 데이터 확인
% disp(size(data));
% disp(data(1:5, 1:5));

% heatmap 생성
figure;
imagesc(theta, phi, data'); % data' 로 transpose하여 축 변경
colorbar;
xlabel('Elevation Angle (Theta) [deg]');
ylabel('Azimuth Angle (Phi) [deg]');
title('RCS Heatmap');
set(gca, 'YDir', 'reverse');

% 색상맵 설정 (최솟값: 파란색, 최댓값: 빨간색)
colormap(jet); % jet colormap은 파란색에서 빨간색으로 변환됨

% 데이터에서 최대값 찾기
% maxValue = max(data(:));

%% FEKO dat파일을 2차원으로 시각화(검증용 좌표 평면에 대하여 직교좌표계형식으로 시각화)

close all
clear;
clc;

folderPath = 'C:/Users/leeyj/Downloads/dat파일 모음/3';
% fileNames = {'KH58_PO_2D_1_9GHz_1.dat', 'Shahed136_PO_2D_1_9GHz_1.dat', 'Tomahawk_PO_2D_1_9GHz_1.dat', ...
%              'AH1_PO_2D_1_9GHz_1.dat', 'F4_PO_2D_1_9GHz_1.dat', '1.dat'};
% fileNames = {'KH58_PO_2D_1_9GHz_2.dat', 'Shahed136_PO_2D_1_9GHz_2.dat', 'Tomahawk_PO_2D_1_9GHz_2.dat', ...
%              'AH1_PO_2D_1_9GHz_2.dat', 'F4_PO_2D_1_9GHz_2.dat', '2.dat'};
fileNames = {'KH58_PO_2D_1_9GHz_3.dat', 'Shahed136_PO_2D_1_9GHz_3.dat', 'Tomahawk_PO_2D_1_9GHz_3.dat', ...
             'AH1_PO_2D_1_9GHz_3.dat', 'F4_PO_2D_1_9GHz_3.dat', '737_PO_2D_1_9GHz_3.dat'};

legendNames = {'KH58', 'Shahed136', 'Tomahawk', 'AH1', 'F4', 'B737'};
colors = {'r', [1, 0.5, 0], [0.6, 0.3, 0], 'g', 'b', 'm'};
figure;
hold on;
title('RCS Plot of Surface #3');
xlabel('Angles [degrees]');
ylabel('RCS [dBsm]');

for i = 1:length(fileNames)
    filePath = fullfile(folderPath, fileNames{i});
    data = readmatrix(filePath, 'FileType', 'text');
    plot(data(:, 1), data(:, 2), 'DisplayName', legendNames{i}, 'Color', colors{i});
end

legend show;
grid on
hold off;

%% dat 파일 하나를 극 좌표계로 시각화

file_path = 'C:/Users/leeyj/Downloads/dat파일 모음/2/F4_PO_2D_1_9GHz_2.dat';
fileID = fopen(file_path, 'r');
data = textscan(fileID, '%f%f', 'HeaderLines', 2);
fclose(fileID);
theta_deg = data{1};
rcs_dbsm = data{2};
theta_rad = deg2rad(theta_deg);

figure;
polarplot(theta_rad, rcs_dbsm, 'LineWidth', 2);
title('RCS Polar Plot');
ax = gca;
ax.ThetaZeroLocation = 'top'; % 0도를 위쪽으로 설정
ax.ThetaDir = 'clockwise'; % 각도가 시계 방향으로 증가
legend('RCS vs Theta');

%%

w = 20;
h = 12;
lambda = (9*10^9)/(3*10^8);

sigma = (4*pi*w^2*h^2)/lambda;