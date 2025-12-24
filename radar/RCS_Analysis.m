clear;clc;close all;

%% Raw data
% 파일 경로 설정 (로컬 컴퓨터에 복사한 경우)
file_path_001 = 'C:\Users\leeyj\OneDrive - 인하대학교\School\Matlab_Files\RADAR\source\RCS_Analysis\RCS_Table\UAV_PO_2D_PEC_0.01.dat';
file_path_01 = 'C:\Users\leeyj\OneDrive - 인하대학교\School\Matlab_Files\RADAR\source\RCS_Analysis\RCS_Table\UAV_PO_2D_PEC_0.1.dat';
file_path_1 = 'C:\Users\leeyj\OneDrive - 인하대학교\School\Matlab_Files\RADAR\source\RCS_Analysis\RCS_Table\UAV_PO_2D_PEC_1.dat';
file_path_2 = 'C:\Users\leeyj\OneDrive - 인하대학교\School\Matlab_Files\RADAR\source\RCS_Analysis\RCS_Table\UAV_PO_2D_PEC_2.dat';
file_path_al_001 = 'C:\Users\leeyj\OneDrive - 인하대학교\School\Matlab_Files\RADAR\source\RCS_Analysis\RCS_Table\UAV_PO_2D_Aluminum_0.01.dat';
file_path_al_01 = 'C:\Users\leeyj\OneDrive - 인하대학교\School\Matlab_Files\RADAR\source\RCS_Analysis\RCS_Table\UAV_PO_2D_Aluminum_0.1.dat';
file_path_al_1 = 'C:\Users\leeyj\OneDrive - 인하대학교\School\Matlab_Files\RADAR\source\RCS_Analysis\RCS_Table\UAV_PO_2D_Aluminum_1.dat';
file_path_al_2 = 'C:\Users\leeyj\OneDrive - 인하대학교\School\Matlab_Files\RADAR\source\RCS_Analysis\RCS_Table\UAV_PO_2D_Aluminum_2.dat';

% 데이터 불러오기
data_001 = importdata(file_path_001);
data_01 = importdata(file_path_01);
data_1 = importdata(file_path_1);
data_2 = importdata(file_path_2);
al_data_001 = importdata(file_path_al_001);
al_data_01 = importdata(file_path_al_01);
al_data_1 = importdata(file_path_al_1);
al_data_2 = importdata(file_path_al_2);

figure(1)
plot(data_001.data(:,1),data_001.data(:,2),data_01.data(:,1),data_01.data(:,2),data_1.data(:,1),data_1.data(:,2),data_2.data(:,1),data_2.data(:,2))
xlim([0 360]);
legend('0.01','0.1','1','2')
ylabel('RCS Value [dBSM]')
xlabel('Azimuth [deg]')
title('Comparing RAW RCS Value of Various Angle Size in PEC')
grid on
grid minor

figure(2)
plot(al_data_001.data(:,1),al_data_001.data(:,2),al_data_01.data(:,1),al_data_01.data(:,2),al_data_1.data(:,1),al_data_1.data(:,2),al_data_2.data(:,1),al_data_2.data(:,2))
xlim([0 360]);
legend('0.01','0.1','1','2')
ylabel('RCS Value [dBSM]')
xlabel('Azimuth [deg]')
title('Comparing RAW RCS Value of Various Angle Size in Aluminum Coating')
grid on
grid minor

figure(3)
plot(data_001.data(:,1),data_001.data(:,2),al_data_001.data(:,1),al_data_001.data(:,2))
xlim([0 360]);
legend('PEC','Aluminum')
ylabel('RCS Value [dBSM]')
xlabel('Azimuth [deg]')
title('Comparing RAW RCS Value PEC with Aluminum (Angle size: 0.01[deg])')
grid on
grid minor

figure(4)
plot(data_01.data(:,1),data_01.data(:,2),al_data_01.data(:,1),al_data_01.data(:,2))
xlim([0 360]);
legend('PEC','Aluminum')
ylabel('RCS Value [dBSM]')
xlabel('Azimuth [deg]')
title('Comparing RAW RCS Value PEC with Aluminum (Angle size: 0.1[deg])')
grid on
grid minor

figure(5)
plot(data_1.data(:,1),data_1.data(:,2),al_data_1.data(:,1),al_data_1.data(:,2))
xlim([0 360]);
legend('PEC','Aluminum')
ylabel('RCS Value [dBSM]')
xlabel('Azimuth [deg]')
title('Comparing RAW RCS Value PEC with Aluminum (Angle size: 1 [deg])')
grid on
grid minor

figure(6)
plot(data_2.data(:,1),data_2.data(:,2),al_data_2.data(:,1),al_data_2.data(:,2))
xlim([0 360]);
legend('PEC','Aluminum')
ylabel('RCS Value [dBSM]')
xlabel('Azimuth [deg]')
title('Comparing RAW RCS Value PEC with Aluminum (Angle size: 2 [deg])')
grid on
grid minor

figure(7)
polarplot(deg2rad(data_001.data(:,1)),data_001.data(:,2),deg2rad(al_data_001.data(:,1)),al_data_001.data(:,2))
%legend('PEC','Aluminium')
title('Comparing RAW RCS Value PEC with Aluminum (Angle size: 0.01[deg])')
grid on
grid minor

figure(8)
polarplot(deg2rad(data_01.data(:,1)),data_01.data(:,2),deg2rad(al_data_01.data(:,1)),al_data_01.data(:,2))
%legend('PEC','Aluminium')
title('Comparing RAW RCS Value PEC with Aluminum (Angle size: 0.1[deg])')
grid on
grid minor

figure(9)
polarplot(deg2rad(data_1.data(:,1)),data_1.data(:,2),deg2rad(al_data_1.data(:,1)),al_data_1.data(:,2))
%legend('PEC','Aluminium')
title('Comparing RAW RCS Value PEC with Aluminum (Angle size: 1 [deg])')
grid on
grid minor

figure(10)
polarplot(deg2rad(data_2.data(:,1)),data_2.data(:,2),deg2rad(al_data_2.data(:,1)),al_data_2.data(:,2))
%legend('PEC','Aluminium')
title('Comparing RAW RCS Value PEC with Aluminum (Angle size: 2 [deg])')
grid on
grid minor

%% Define Degree
degree = 0:0.001:360;
%% Moving Average Filter(3deg) + interpolation(논문)
maf_deg = 3; % Filtering Degree of Moving Average Filter 
RCS_MAF_I_001 = interp1(al_data_001.data(:,1),Moving_Average_Filter(al_data_001.data(:,2),maf_deg),degree);
RCS_MAF_I_01 = interp1(al_data_01.data(:,1),Moving_Average_Filter(al_data_01.data(:,2),maf_deg),degree);
RCS_MAF_I_1 = interp1(al_data_1.data(:,1),Moving_Average_Filter(al_data_1.data(:,2),maf_deg),degree);
%% Gaussian Filter(sigma:1.5) + interpolation
gf_sigma = 1.5; % Standard Deviation of Gaussian Filter
RCS_GF_I_001 = interp1(al_data_001.data(:,1),Gaussian_Filter(al_data_001.data(:,2),gf_sigma),degree);
RCS_GF_I_01 = interp1(al_data_01.data(:,1),Gaussian_Filter(al_data_01.data(:,2),gf_sigma),degree);
RCS_GF_I_1 = interp1(al_data_1.data(:,1),Gaussian_Filter(al_data_1.data(:,2),gf_sigma),degree);
%% Moving Average Filter(3deg) + Gaussian Noise + interpolation
n_maf_deg = 3; % Filtering Degree of Moving Average Filter
maf_noise_std = 1.5; % Standard Deviation of Gaussian Noise
RCS_MAF_N_I_001 = interp1(al_data_001.data(:,1),Gaussian_Noise(Moving_Average_Filter(al_data_001.data(:,2),n_maf_deg),maf_noise_std),degree);
RCS_MAF_N_I_01 = interp1(al_data_01.data(:,1),Gaussian_Noise(Moving_Average_Filter(al_data_01.data(:,2),n_maf_deg),maf_noise_std),degree);
RCS_MAF_N_I_1 = interp1(al_data_1.data(:,1),Gaussian_Noise(Moving_Average_Filter(al_data_1.data(:,2),n_maf_deg),maf_noise_std),degree);
%% Gaussian Filter(sigma:1.5) + Gaussian Noise + interpolation
n_gf_sigma = 1.5; % Standard Deviation of Gaussian Filter
gf_noise_std = 1.5; % Standard Deviation of Gaussian Noise
RCS_GF_N_I_001 = interp1(al_data_001.data(:,1),Gaussian_Noise(Gaussian_Filter(al_data_001.data(:,2),n_gf_sigma),gf_noise_std),degree);
RCS_GF_N_I_01 = interp1(al_data_01.data(:,1),Gaussian_Noise(Gaussian_Filter(al_data_01.data(:,2),n_gf_sigma),gf_noise_std),degree);
RCS_GF_N_I_1 = interp1(al_data_1.data(:,1),Gaussian_Noise(Gaussian_Filter(al_data_1.data(:,2),n_gf_sigma),gf_noise_std),degree);
%%

figure(10)

plot(al_data_1.data(:,1),al_data_1.data(:,2),degree,RCS_MAF_N_I_1,degree,RCS_GF_N_I_1)
legend('raw','gf','gfn')
ylabel('RCS Value [dBSM]')
xlabel('Azimuth [deg]')

title('filtered')
grid on
grid minor
%%

figure(11)
plot(al_data_01.data(:,1),filtered_al_data_01,al_data_1.data(:,1),filtered_al_data_1)
legend('0.1','1')
ylabel('RCS Value [dBSM]')
xlabel('Azimuth [deg]')
title('filtered')
grid on
grid minor

figure(12)
plot(al_data_1.data(:,1),al_data_1.data(:,2),al_data_1.data(:,1),filtered_al_data_1)
legend('Aluminium','filtered 1')
ylabel('RCS Value [dBSM]')
xlabel('Azimuth [deg]')
title('0.01 comparison pec vs al')
grid on
grid minor

figure(13)
plot(al_data_1.data(:,1),al_data_1.data(:,2),al_data_1.data(:,1),filtered_al_data_1,al_data_1.data(:,1),g_filtered_al_data_1)
legend('Aluminium','filtered 1','gs')
ylabel('RCS Value [dBSM]')
xlabel('Azimuth [deg]')
title('0.01 comparison pec vs al')
grid on
grid minor

figure(14)
plot(al_data_1.data(:,1),filtered_al_data_1,al_data_1.data(:,1),g_filtered_al_data_1)
legend('Aluminium','filtered 1','gs')
ylabel('RCS Value [dBSM]')
xlabel('Azimuth [deg]')
title('0.01 comparison pec vs al')
grid on
grid minor
%% MSE


mse_001 = mean((data_001.data(:,2)-al_data_001.data(:,2)).^2)
mse_01 = mean((data_01.data(:,2)-al_data_01.data(:,2)).^2)
mse_1 = mean((data_1.data(:,2)-al_data_1.data(:,2)).^2)
mse_2 = mean((data_2.data(:,2)-al_data_2.data(:,2)).^2)


%%
