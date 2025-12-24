close all
clc;
clear;

data = load('boxMResults.mat');
Sth = data.Sth;

%% 수식에 따라 SNR 산출

% scanter 4000의 경우
lambda = freq2wavelen(9e9);         % Wavelength (m), 레이더 주파수 입력
Pt = 12e3;                         % Peak power (W), 12kW
tau = 8.0e-8;                       % Pulse width (s), scanter 4000의 경우 80ns
G = 34;                             % Transmit and receive antenna gain (dB)
Ts = 290;                  % System temperature (K) 
% rcs = 9;
rcs_min = 10^(-30/10);  % 최소 RCS 값 (선형 스케일로 변환)
rcs_max = 10^(47.05/10); % 최대 RCS 값 (선형 스케일로 변환)
Rm = 100e3;                         % Required maximum range (m)
L = 0;                              % Combined transmission line and propagation losses (dB)
R = (1:1:120e3).';                 % Range samples (m)

%% SNR을 결과로 내보냄
SNR = radareqsnr(lambda,R,Pt,tau,'Gain',G,'Ts',Ts,'RCS',rcs,'Loss',L);

%% Plot the maximum range requirement together with the computed available SNR

radarmetricplot(R*1e-3,SNR,'MetricName','Available SNR','MaxRangeRequirement',Rm*1e-3,...
    'RangeUnit','km','MetricUnit','dB','RadarName',{'Surveillance Radar'});
legend('Location','best');

%% POfacets 해석을 통해 생성된 mat 파일에 대해 고각과 방위각을 입력받아 그 때의 RCS를 이용하여 위의 그래프를 그림

azimuth = input('방위각(0 ~ 360도) 값을 입력하세요: ');
elevation = input('고각(0 ~ 180도) 값을 입력하세요: ');
if azimuth < 0 || azimuth > 360 || elevation < 0 || elevation > 180
    error('입력한 방위각 또는 고각 값이 범위를 벗어났습니다.');
end

azimuth_index = azimuth + 1;
elevation_index = elevation + 1;

resultRCSdBsm = Sth(azimuth_index, elevation_index);
resultRCSm2 = 10^(resultRCSdBsm/10);    % dBsm에서 m^2으로 변환함
resultSNR = radareqsnr(lambda,R,Pt,tau,'Gain',G,'Ts',Ts,'RCS',resultRCSm2,'Loss',L);
% fprintf('방위각 %d도, 고각 %d도에서의 RCS[dBsm] 값은: %.4f\n', azimuth, elevation, resultRCS);
radarmetricplot(R*1e-3,resultSNR,'MetricName','Available SNR','MaxRangeRequirement',Rm*1e-3,...
    'RangeUnit','km','MetricUnit','dB','RadarName',{'Surveillance Radar'});
legend('Location','best');

%% "탐지됨"이라 할수있는 최소 SNR을 계산

% probability of detetection과 maximum acceptable probability of false alarm을 계산
% 계산결과로 나온 D0, D1들은 목표물 포착을 위한 최소한의 SNR 값
Pd = 0.9;       % probability of detetection
Pfa = 1e-6;     % maximum acceptable probability of false alarm
D0 = detectability(Pd,Pfa,1,'Swerling0');

% N은 레이더가 목표물을 탐지하기 위해 방출하는 독립적인 펄스의 수
% 펄스의 수가 많을수록(N이 클수록) 목표물 탐지에 필요한 신호 대 잡음비를 더 쉽게 확보
N = 10;
DN = detectability(Pd,Pfa,N,'Swerling1');

% SNR이 고정되어있을때(Swerling 0) 단일 펄스와 여러 펄스를 사용할 때의 차이로 이득을 정의
Gi = detectability(Pd,Pfa,1,'Swerling0') - detectability(Pd,Pfa,N,'Swerling0');

% The fluctuation loss는 SNR required to detect a fluctuating target과 SNR required to detect a steady target간의 차이
Lf = detectability(Pd,Pfa,N,'Swerling1') - detectability(Pd,Pfa,N,'Swerling0');

%% radarbudgetplot

% radarbudgetplot function illustrates the components of the detectability factor
radarbudgetplot([DN -Gi Lf], {'Single-pulse steady target','Pulse integration gain','Fluctuation loss'});
title('Detectability Factor')

% Substitute the detectability factor into the range form of the radar equation 
% as the minimum required SNR to evaluate the actual maximum range of the system.
% radareqrng(lambda,DN,Pt,tau,'Gain',G,'Ts',Ts,'RCS',rcs,'Loss',L,'unitstr','km')

disp('레이더 시스템의 최대 탐지 거리 RE: ');
% 레이더 방정식(radar range equation)을 이용하여 시스템의 최대 탐지 거리를 계산
RE = radareqrng(lambda,DN,Pt,tau,'Gain',G,'Ts',Ts,'RCS',rcs,'Loss',L,'unitstr','km')

%% 레이더와 목표물간의 거리와 SNR에 따라 탐지 가능 여부를 확인

radarmetricplot(R*1e-3,SNR,DN, ...
    'MetricName','Available SNR', ...
    'RequirementName','Detectability', ...
    'MaxRangeRequirement',Rm*1e-3, ...
    'RangeUnit','km','MetricUnit','dB', ...
    'ShowStoplight',true, ...
    'RadarName',{'Surveillance Radar'});
title([{'Available SNR vs Range'}, {'(No Losses)'}]);
ylim([-10 270]);
legend('Location','best');

%% Eclipsing

Rmin = time2range(tau);

prf = 9000;                         % Pulse repetition frequency
Rua = time2range(1/prf);

Du = tau*prf;                       % Duty cycle
Fecl = eclipsingfactor(R,Du,prf);   % Eclipsing factor

SNR = radareqsnr(lambda,R,Pt,tau,'Gain',G,'Ts',Ts,'RCS',rcs,'CustomFactor',Fecl,'Loss', L);

radarmetricplot(R*1e-3,SNR,DN, ...
    'MetricName','Available SNR', ...
    'RequirementName','Detectability', ...
    'MaxRangeRequirement',Rm*1e-3, ...
    'RangeUnit','km','MetricUnit','dB', ...
    'ShowStoplight',true, ...
    'RadarName',{'Surveillance Radar'});
title([{'Available SNR vs Range'}, {'(With Eclipsing)'}]);
legend('Location','best');

%% SNR에 따른 Eclipsing 현상 비교
% 최대 최소 rcs로 거리에 따른 SNR을 도출
Rmin = time2range(tau);
prf = 2000;     % 펄스 반복 주파수를 높일수록 이클립싱 현상이 더 자주 발생함
% 7가지 레이더 PRF를 토대로 2000 Hz를 사용
% DWSR 2001X: 200-2400 Hz, user selectable
% MODEL 1623: 3000 Hz nominal 
% SU70-14E: 2000 Hz
% The National Centre for Atmospheric Science Atmospheric Measurement Facility's (NCAS AMF) mobile X-band radar: 250-2000 Hz
% NIED MP-X: 1800 Hz
% Specifications of Solid-State X-Band Dual Polarization Doppler Weather Radar: 900-3000Hz
Rua = time2range(1/prf);
Du = tau*prf;
Fecl = eclipsingfactor(R,Du,prf);
SNR_min = radareqsnr(lambda,R,Pt,tau,'Gain',G,'Ts',Ts,'RCS',rcs_min,'CustomFactor',Fecl,'Loss', L);
SNR_max = radareqsnr(lambda,R,Pt,tau,'Gain',G,'Ts',Ts,'RCS',rcs_max,'CustomFactor',Fecl,'Loss', L);

% 시각화
close all;
Target_Range = linspace(0, 120000, 120000)';

figure;
plot(Target_Range/1000, SNR_max, 'b-', 'LineWidth', 1.5);
hold on;
plot(Target_Range/1000, SNR_min, 'b--', 'LineWidth', 1.5);
hold off;

title('Available SNR vs Range (with Eclipsing)', 'FontSize', 10);
xlabel('Target Range [km]', 'FontSize', 8);
ylabel('Available SNR (dB)', 'FontSize', 8);
legend({'X-Band Max', 'X-Band Min'}, 'FontSize', 8, 'Location', 'Best');
grid on;
set(gca, 'FontSize', 12);

%%
figure;
hold on;

radarmetricplot(R * 1e-3, SNR_min, DN, ...
    'MetricName', 'Available SNR', ...
    'RequirementName', 'Detectability', ...
    'MaxRangeRequirement', Rm * 1e-3, ...
    'RangeUnit', 'km', 'MetricUnit', 'dB', ...
    'ShowStoplight', false, ...
    'RadarName', {'Surveillance Radar'});
title([{'Available SNR vs Range (With Eclipsing)'}, {'(Min and Max RCS)'}]);

radarmetricplot(R * 1e-3, SNR_max, DN, ...
    'MetricName', 'Available SNR', ...
    'RequirementName', 'Detectability', ...
    'MaxRangeRequirement', Rm * 1e-3, ...
    'RangeUnit', 'km', 'MetricUnit', 'dB', ...
    'ShowStoplight', false, ...
    'RadarName', {'Surveillance Radar'});

%% STC(Sensitivity Time Control)

Rstc = 60e3;                        % STC cutoff range (m), 60e3(m)미터 안에 들어오는 SNR 값을 cut off 시킴
Xstc = 4;                           % STC exponent selected to maintain target detectability at ranges below Rstc (since the signal power is inversely proportional to R^4)
Fstc = stcfactor(R,Rstc,Xstc);      % STC factor

SNR = radareqsnr(lambda,R,Pt,tau,'Gain',G,'Ts',Ts,'RCS',rcs,'CustomFactor',Fecl+Fstc,'Loss',L);

radarmetricplot(R*1e-3,SNR,DN, ...
    'MetricName','Available SNR', ...
    'RequirementName','Detectability', ...
    'MaxRangeRequirement',Rm*1e-3, ...
    'RangeUnit','km','MetricUnit','dB', ...
    'ShowStoplight',true, ...
    'RadarName',{'Surveillance Radar'});
title([{'Available SNR vs Range'}, {'(With STC and Eclipsing for 1 m^2 Target)'}]);
legend('Location','best');
ylim([-30 30])

%% STC of RCS of 0.03 m^2 target

SNRsmallRCS = radareqsnr(lambda,R,Pt,tau,'Gain',G,'Ts',Ts,'RCS',0.03,'CustomFactor',Fecl+Fstc,'Loss',L);

radarmetricplot(R*1e-3,SNRsmallRCS,DN, ...
    'MetricName','Available SNR', ...
    'RequirementName','Detectability', ...
    'MaxRangeRequirement',Rm*1e-3, ...
    'RangeUnit','km','MetricUnit','dB', ...
    'ShowStoplight',true, ...
    'RadarName',{'Surveillance Radar'});
title([{'Available SNR vs Range'}, {'(With STC and Eclipsing for 0.03 m^2 Target)'}]);
legend('Location','best');
ylim([-30 20])

%% Scanning

% Beam Shape Loss
Lb = beamloss;
beamloss(true);
% Scan Sector Loss
theta = [-60 60];
Larray = arrayscanloss(Pd,Pfa,N,theta,'Swerling1');

%% MTI(Moving Target Indicator) and CFAR(Constant False Alarm Rate)
% Signal Processing

% MTI
m = 2;
[Lmti_a, Lmti_b] = mtiloss(Pd,Pfa,N,m,'Swerling1');

% Binary Integration
M = 6;
Lbint = binaryintloss(Pd,Pfa,N,M);

% CFAR
Nrc = 120;
Lcfar = cfarloss(Pfa,Nrc);

%% Effective Detectability Factor

D = [D0 -Gi Lf Lmti_a+Lmti_b Lcfar];
radarbudgetplot(D, {'Single-pulse steady target','Pulse integration gain','Fluctuation loss'...
    'MTI loss', 'CFAR loss',});
title('Detectability Factor')

%% mat파일에서 최대값과 최소값 RCS를

filePath = 'C:/Users/leeyj/lab_ws/data/VTD/Result_8GHz_360.mat';
data = load(filePath);
if isfield(data, 'Sth')
    Sth = data.Sth;
    maxRCS = max(Sth(:)); % Maximum value in Sth
    minRCS = min(Sth(:)); % Minimum value in Sth
    fprintf('Maximum RCS value: %.2f dBsm\n', maxRCS);
    fprintf('Minimum RCS value: %.2f dBsm\n', minRCS);
else
    error('The field "Sth" does not exist in the loaded .mat file.');
end







