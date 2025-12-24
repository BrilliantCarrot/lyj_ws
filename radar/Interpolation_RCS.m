% FEKO를 통해 해석된 RCS 데이터를 학습을 위한 환경 조성을 위해 보간
%% << 고각 Phi 고정 방위각 Theta 변동 >>
%% 데이터 불러오기

data = importdata("Shahed136_PO_3D_PEC_1.dat");
data = data.data;

%% 2차원 RCS 데이터 plot

figure
plot(data(:, 1), data(:, 2), 'LineWidth', .5);
xlabel('Plane Wave Phi');
ylabel('Far Field');
title('토마호크, LE-PO, 8GHz');

%% 데이터에 보간법 적용
% 스플라인 보간법 적용

theta_rad = data(:, 1);
y = data(:, 2);
continuousX = min(theta_rad):0.01:max(theta_rad);
interpY = interp1(theta_rad, y, continuousX, 'spline');

%% 보간법이 적용된 2차원 RCS 데이터 plot

figure
plot(continuousX, interpY, 'LineWidth', 1, 'DisplayName', 'Spline Interpolation');
xlabel('Plane Wave Phi');
ylabel('Far Field');
title('spline 보간법 적용, 토마호크, LE-PO, 8GHz');

%% << 고각 Phi 및 방위각 Theta 변동 >>
%% 3차원 RCS 데이터에 보간법 적용
% 데이터 불러오기 & 전처리

% data_3d = importdata("..\\data\\rcs_table\\UAV_PO_8GHz_3D.dat");
% data_3d = importdata("..\\data\\rcs_table\\Tomahawk_LE-PO_8GHz_3D.dat");
% data_3d = importdata("..\\data\\rcs_table\\AH-1_LE-PO_8GHz_3D.dat");
data_3d = importdata("Shahed136_PO_3D_PEC_1.dat");
data_3d = data_3d.data;

theta = data_3d(:, 1);
phi = data_3d(:, 2);
value = data_3d(:, 3);

theta_rad = deg2rad(theta);
phi_rad = deg2rad(phi);

% rcs_filename = sprintf("C:\\Users\\leeyj\\OneDrive - 인하대학교\\School\\Matlab_Files\\RADAR\\data\\rcs_table\\F-4_3D.csv");
% rcsData = readmatrix(rcs_filename);

%% scatterdInterpolant 함수로 RCS 값을 보간

% if isrow(theta_rad)
%     theta_rad = theta_rad';
% end
% if isrow(phi_rad)
%     phi_rad = phi_rad';
% end
% if isrow(value)
%     value = value';
% end
% 
% F = scatteredInterpolant(theta_rad, phi_rad, value, 'natural');
% 
% theta_interp = linspace(min(theta_rad), max(theta_rad), 1801);
% phi_interp = linspace(min(phi_rad), max(phi_rad), 3601);
% [theta_interp_mesh, phi_interp_mesh] = meshgrid(theta_interp, phi_interp);
% 
% value_interp = F(theta_interp_mesh, phi_interp_mesh);
% new_interp = value_interp';

%% interp2 함수로 RCS 값을 보간

value_matrix = reshape(value, [361,181]);
value_matrix = value_matrix';
phi = linspace(0, 180, 181);
theta = linspace(0, 360, 361);
[Theta, Phi] = meshgrid(theta, phi);
phiq = linspace(0, 180, 1801);
thetaq = linspace(0, 360, 3601);
[Thetaq, Phiq] = meshgrid(thetaq, phiq);
new_matrix = interp2(Theta, Phi, value_matrix, Thetaq, Phiq, 'cubid');

% % Create a grid for interpolation
% [theta_grid, phi_grid] = meshgrid(linspace(min(theta_rad), max(theta_rad), 1801), linspace(min(phi_rad), max(phi_rad), 3601));
% % Reshape the value data into a matrix that matches the size of the theta and phi grids
% value_matrix = reshape(value, size(theta_grid));
% % Interpolate the data using interp2
% value_interp = interp2(theta_grid, phi_grid, value_matrix, theta_rad, phi_rad, 'linear');
% new_interp = value_interp';

% theta_interp = linspace(min(theta_rad), max(theta_rad), 1801);
% phi_interp = linspace(min(phi_rad), max(phi_rad), 3601);
% [theta_grid, phi_grid] = meshgrid(theta_interp, phi_interp);
% value_interp = interp2(theta_grid, phi_grid, rcsData, theta_rad, phi_rad, 'linear');
% new_interp = value_interp';

% [Phi, Theta] = meshgrid(linspace(min(phi_rad),max(phi_rad),1801), linspace(min(theta_rad),max(theta_rad),3601));
% value_matrix = reshape(value, size(theta_grid));
% rcs_value = interp2(Phi, Theta, value_matrix, phi_rad, theta_rad, "linear");
% new_interp = value_interp';

% % Reshape the value data into a matrix that matches the size of the theta and phi grids
% value_matrix = reshape(value, [361,181]);
% value_matrix = value_matrix';
% % % Create a grid for interpolation
% [phi_grid, theta_grid] = meshgrid(0:360, 0:180);
% % Interpolate the data using interp2
% theta_interp = linspace(min(theta), max(theta), 1801);
% phi_interp = linspace(min(phi), max(phi), 3601);

%% 불러온 dat 파일을 그대로 csv 파일로 만듦

F = scatteredInterpolant(theta_rad, phi_rad, value, 'natural');
theta_interp = linspace(min(theta_rad), max(theta_rad), 181);
phi_interp = linspace(min(phi_rad), max(phi_rad), 361);
[theta_interp_mesh, phi_interp_mesh] = meshgrid(theta_interp, phi_interp);
value_interp = F(theta_interp_mesh, phi_interp_mesh);
new_interp = value_interp';

%% 보간 전 및 보간 후 데이터 plot

figure;
subplot(1,2,1);
scatter3(theta_rad, phi_rad, value, 'filled','MarkerFaceAlpha',0.3);
xlabel('Theta (rad)');
ylabel('Phi (rad)');
zlabel('FarField1');
title('Spherical coordinate');

% subplot(1,2,2);
% [x,y,z] = sph2cart(phi_rad,theta_rad,value);
% scatter3(x,y,z,1,"filled",'MarkerFaceAlpha',0.3);
% xlabel('Theta (rad)');
% ylabel('Phi (rad)');
% zlabel('FarField1');
% title('Cartesian Coordinate');

subplot(1, 2, 2);
surf(theta_interp_mesh, phi_interp_mesh, value_interp);
xlabel('Theta (rad)');
ylabel('Phi (rad)');
zlabel('Interpolated FarField1');
title('Interpolated Data');


%% csv파일 쓰기

% writematrix(new_interp, "UAV_3D.csv");
% writematrix(new_interp, "Tomahawk_3D.csv");
% writematrix(new_interp, "AH-1_3D.csv");
writematrix(new_interp, "Shahed136_PO_3D_PEC_1.csv");
% dlmwrite('tomahawk_3D.txt',tomahawk_3d,'delimiter','\t');

%% 테스트

[xq, yq] = ndgrid(linspace(min(theta_rad), max(theta_rad), 1800), linspace(min(phi_rad), max(phi_rad), 100));
vq = F(xq, yq);

% elevation_grid = linspace(0, pi, 18000); % Adjust the number of points as needed
% azimuth_grid = linspace(0, 2*pi, 36000); % Adjust the number of points as needed
% [elevation_interp, azimuth_interp] = meshgrid(elevation_grid, azimuth_grid);

% Interpolate RCS values using interp2
% rcs_interp = interp2(x, y, z, elevation_interp, azimuth_interp, 'spline'); % or other interpolation method

% % Step 3: Convert interpolated Cartesian coordinates back to spherical coordinates
% [elevation_interp_rad, azimuth_interp_rad] = cart2sph(elevation_interp, azimuth_interp, rcs_interp)

% Define the grid onto which you want to interpolate
[thetaq, phiq] = meshgrid(linspace(min(theta_rad), max(theta_rad), 100), linspace(min(phi_rad), max(phi_rad), 100));

% % Use griddata to interpolate the data onto a uniform grid
vq = griddata(theta_rad, phi_rad, v, thetaq, phiq, 'cubic');
% 
% % Now you can use interp2 with the new uniform grid
% % new_theta = data_3d(:, 1); % new theta value(s) where you want to interpolate
% % new_phi = data_3d(:, 2); % new phi value(s) where you want to interpolate
new_v = interp2(thetaq, phiq, vq, thetaq, phiq);
% 
% % Convert spherical coordinates to Cartesian coordinates for plotting
% [x, y, z] = sph2cart(theta, phi, v);
% [xq, yq, zq] = sph2cart(thetaq, phiq, vq);
% 
% % Create a new figure
% figure;

% % Plot the original data
% subplot(1, 2, 1);
% surf(x, y, z, 'EdgeColor', 'none');
% title('Original Data');
% xlabel('X'); ylabel('Y'); zlabel('V');
% view(3); % 3D view
% axis tight; % Fit the axis tightly to the plot
% grid on; % Add a grid

% % Plot the interpolated data
% subplot(1, 2, 2);
% surf(xq, yq, zq, 'EdgeColor', 'none');
% title('Interpolated Data');
% xlabel('X'); ylabel('Y'); zlabel('V');
% view(3); % 3D view
% axis tight; % Fit the axis tightly to the plot
% grid on; % Add a grid
