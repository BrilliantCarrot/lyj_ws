% 테스트 코드 모음

%% 클러터 적용 코드 완성본

clc; close; 

% 레이더 파라미터
lambda = freq2wavelen(9e9); % Wavelength (m), Scanter 4000
% lambda = freq2wavelen(5.6e9); % Wavelength (m), 레이더 주파수 입력
% Pt = 114.6e3;                   % Peak power (W)
Pt = 12e3;                  % Scanter 4000의 Peak Power
Gt = 34;                    % Transmit antenna gain (dB)
Gr = 34;                    % Receive antenna gain (dB)
rcs_db = 9;                    % 헬기 RCS(dB)
rcs = 10^(rcs_db/10);          % 헬기 RCS, dB값을 log값으로 변환 
% tau = 8.0e-8;               % pulse width, scanter 4000의 경우 80ns

% 노이즈 항
R = linspace(2e3, 50e3, 100).';  % 거리 (0에서 50km)
k = 1.38e-23;               % 볼츠만 상수 (J/K)
Ts = 290;                   % 시스템 잡음 온도 (K)
B = 1e6;                    % Bandwidth (Hz)
F = 10^(6 / 10);                    % Noise Figure (m^2)
L = 10^(10 / 10);                   % Radar Loss (m^2)

sigma_target = 7.9432;      % 목표물 RCS (m^2), 9dB에서 
% sigma_target = 2.5119;      % 목표물 RCS (m^2), 논문의 경우

% 지형의 클러터
% 교재의 계산식을 이용
% 대기의 경우 0.001 m^2으로, 지표면의 경우 5 m^2으로 설정
c = 3e8;                      % 전파 속도 (m/s)
pulse_width = 8.0e-8;         % 펄스 폭 (s), 80 nswaypoint
sigma_0 = 10^(-20/10);        % 클러터 산란(반사)계수 (선형(log) 스케일), -20 dB(Flatland)
% 1도 및 2도는 탐지용 레이더로써 적합한 조합
theta_A = deg2rad(1);       % 방위각 빔폭 (rad)
theta_E = deg2rad(2);       % 고각 빔폭 (rad)
SL_rms = 10^(-20/10);         % 사이드로브의 RMS 수준 (선형(log) 스케일), -20 dB

% 높이 및 거리
% R = 35e3;                             % 단일 거리용 임시 변수
% Rg = R*cos(theta_E);                    % slant range(R)의 지표면 투영
h_t = 200;                             % 목표물 높이 (m)
h_r = 5;                                % 레이더 높이 (m)
% R_s = sqrt(Rg.^2 + (h_t - h_r)^2);      % 높이가 고려된 레이더와 기체 간 Slant Range
R_e = 6.371e6;                          % 지구 반지름 (m)

% Radar Range Resolution(레이더 거리 해상도) 계산 코드
% delta_Rg = c * pulse_width / 2;

% 교재에서 나온 방식의 레이더 거리 해상도 계산
theta_r = asin(min(1, max(-1, h_r ./ R)));          % asin 입력값 제한
theta_e = asin(min(1, max(-1, (h_t - h_r) ./ R)));  % asin 입력값 제한
Rg = R .* cos(theta_r);
delta_R = c * pulse_width / 2;                      % Slant Range의 거리 해상도
delta_Rg = delta_R * cos(theta_r);                  % 지표면 투영 거리 해상도

% 안테나 이득 패턴(가우시안 안테나 패턴 가정하였을 시)
theta_sum = theta_e + theta_r;
G_theta = exp(-2.776 * (theta_sum ./ theta_E).^2);  % 고각 및 방위각 두개에 대한 이득
% 메인빔 클러터 면적 및 RCS 계산
A_MBc = delta_Rg .* Rg * theta_A;
sigma_MBc = sigma_0 .* A_MBc .* G_theta.^2;
% 사이드로브 클러터 면적 및 RCS 계산
A_SLc = delta_Rg .* pi .* Rg;
sigma_SLc = sigma_0 .* A_SLc .* SL_rms.^2;
% 레이다 탐지 범위의 성분 중 지평선 축의 거리
R_h = sqrt((8 * R_e * h_r)/3);

R_s = sqrt(Rg.^2 + (h_t - h_r)^2);      % Slant Range

% 총 클러터 RCS 계산
sigma_TOTc = (sigma_MBc + sigma_SLc) ./ (1 + (R_s / R_h).^4);
sigma_clutter = sigma_TOTc;        % 지형 클러터의 RCS (m^2), 논문의 경우 -20 dB

% dB로부터 선형 스케일로 변환(dB에서 m^2단위로 변환)
Gt_lin = 10^(Gt / 10);
Gr_lin = 10^(Gr / 10);

% SNR, CNR, SCR 정리
SNR = (Pt * Gt_lin * Gr_lin * sigma_target * lambda^2) ./ ((4 * pi)^3 * R.^4 * k * Ts * B * F * L);
SNR_dB = 10 * log10(SNR);
CNR = (Pt * Gt_lin * Gr_lin * sigma_clutter * lambda^2) ./ ((4 * pi)^3 * R.^4 * k * Ts * B * F * L);
CNR_dB = 10 * log10(CNR);
SCR = (Pt * Gt_lin * Gr_lin * sigma_target * lambda^2) ./ (Pt * Gt_lin * Gr_lin * sigma_clutter * lambda^2);
SCR_dB = 10 * log10(SCR);
SIR = 1./((1./SNR)+(1./SCR));       % 클러터의 영향이 고려된 목표물의 SNR 값을 SIR(SCNR)로 정의
SIR_dB = 10 * log10(SIR);

figure;
hold on;
plot(R / 1e3, SNR_dB, 'b-','LineWidth', 1.5);
plot(R / 1e3, CNR_dB, 'k-', 'LineWidth', 1.5);
plot(R / 1e3, SCR_dB, 'g-', 'LineWidth', 1.5);
plot(R / 1e3, SIR_dB, 'r-', 'LineWidth', 1.5);
xlabel('Rs (Slant Range) in Km');
ylabel('dB');
title('2km 상공 항공기의 경우 클러터의 영향이 고려된 SNR 수치');
legend("SNR", "CNR", "SCR", "SIR")
grid on;

% Pc = (Pt * Gt_lin * Gr_lin * sigma_clutter * lambda^2) ./ ((4 * pi)^3 * R.^4);    % 클러터 파워 (Pc)
% Pn = k * Ts * B;    % 노이즈 파워 (Pn)
% CNR = Pc / Pn;
% CNR_dB = 10 * log10(CNR);
% Ptgt = (Pt * Gt_lin * Gr_lin * sigma_target * lambda^2) ./ ((4 * pi)^3 * R.^4);   % 타겟의 파워 (Ptgt)
% SCR = Ptgt / Pc;
% SCR_dB = 10 * log10(SCR);

% SCR 및 CNR에 따라 레이더에서의 탐지를 위한 추가 SNR을 계산
% required_SNR_cluttered_dB = 10 * log10(1 + CNR / SCR);
% required_SNR_without_CNR = 10 * log10(1 + SCR);
% fprintf('Clutter-to-Noise Ratio: %.2f dB\n', CNR_dB);
% fprintf('Signal-to-Clutter Ratio: %.2f dB\n', SCR_dB);
% fprintf('CNR과 SCR을 전부 고려하였을 시 요구되는 추가 SNR: %.2f dB\n', required_SNR_cluttered_dB);
% fprintf('CNR을 고려 안하였을 시 요구되는 추가 SNR: %.2f dB\n', required_SNR_without_CNR);


%% 시뮬레이션을 위한 DTED 데이터 생성

% CSV 파일 경로
file_path = 'C:/Users/leeyj/lab_ws/data/VTD/지형 데이터/target_lotation_DTED_LCC.csv';

% 파일 열기
fileID = fopen(file_path, 'r');

% 파일에서 데이터 읽기: 따옴표와 쉼표를 처리
raw_data = textscan(fileID, '%s', 'Delimiter', '\n'); % 각 줄을 문자열로 읽기
fclose(fileID);

% 문자열 데이터를 쉼표로 분리하고 숫자로 변환
split_data = cellfun(@(x) str2double(strsplit(x(2:end-1), ',')), raw_data{1}, 'UniformOutput', false);
numeric_data = cell2mat(split_data);

% 열 분리: X, Y, 고도
x_coords = numeric_data(:, 1);
y_coords = numeric_data(:, 2);
elevation = numeric_data(:, 3);

% 데이터 확인
disp('X, Y, Elevation data loaded successfully:');
disp([x_coords(1:5), y_coords(1:5), elevation(1:5)]);

% 3D 지형 시각화 (예제)
[X, Y] = meshgrid(unique(x_coords), unique(y_coords));
Z = griddata(x_coords, y_coords, elevation, X, Y, 'linear'); % 고도 데이터 보간

figure;
surf(X, Y, Z, 'EdgeColor', 'none');
colormap('terrain');
colorbar;
xlabel('X Coordinate (meters)');
ylabel('Y Coordinate (meters)');
zlabel('Elevation (meters)');
title('3D Terrain Visualization');
