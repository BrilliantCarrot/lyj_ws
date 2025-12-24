% 파일 경로 설정
filename = 'C:/Users/leeyj/OneDrive - 인하대학교/School/Matlab_Files/RADAR/data/rcs_table/diff_csv.csv';

% CSV 파일에서 데이터 읽기
data = csvread(filename);

% 고각 (Elevation angle)과 방위각 (Azimuth angle) 생성
elevation = 0:180; % 0도부터 180도까지
azimuth = 0:360;   % 0도부터 360도까지

% Meshgrid 생성
[Azimuth, Elevation] = meshgrid(azimuth, elevation);

% Surf 함수로 3D 그래프 그리기
figure;
surf(Elevation, Azimuth, data);

% 그래프 제목 및 축 레이블 설정
title('RCS Error Matlab vs FEKO');
xlabel('Azimuth Angle (degrees)');
ylabel('Elevation Angle (degrees)');
zlabel('RCS Error (dB)');
colormap(jet);

% 그래프의 시각적 품질 향상
shading interp;
colorbar;
view(2); % 위에서 바라본 2D 시각화, 3D 시각화를 원하면 이 줄을 주석처리

% 그래프를 3D로 시각화하려면 아래 view 설정 사용
% view(3); % 3D 시각화