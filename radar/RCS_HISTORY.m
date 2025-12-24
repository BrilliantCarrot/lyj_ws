%% RCS History

clc; clear

% Data Input
scernario_filename = 'fixed wing scenario1.csv';
rcs_filename = 'F-4_Phantom_3D.csv';
rawData = readtable(scernario_filename,"VariableNamingRule","preserve");
rcsData = readmatrix(rcs_filename);

% Aircraft LLA Coordinate
lla0 = [rawData.Latitude,rawData.Longitude,rawData.Altitude];

% Radar LLA Coordinate(Last LLA coordinate of Aircraft)
lla = lla0(length(lla0),:) ;

% Euler Angle
euler = [rawData.Roll,rawData.Pitch,rawData.Yaw];

% Compute Line of Sight(NED-Frame)
LOS_NED = lla2ned(lla,lla0,'ellipsoid');

% Euler Angle Conversion(Radian to degree)
roll = deg2rad(euler(:,1));
pitch = deg2rad(euler(:,2));
yaw = deg2rad(euler(:,3));

% Compute DCM(NED-Frame to B-frame)
dcm_n2b = angle2dcm(yaw,pitch,roll);

loop_result = [];
RCS_history = [];

for i = 1:(length(lla0)-1) 
    % Compute Line of Sight(B-Frame)
    LOS_B = dcm_n2b(:,:,i)*LOS_NED(i,:)';
    
    % Calculate Azimuth (0 ~ 360 [deg])
    psi = round(mod(rad2deg(atan2(LOS_B(2),LOS_B(1))),360),1);

    % Calculate Elevation (0 ~ 180 [deg])
    theta = round(mod(rad2deg(atan2(LOS_B(3), hypot(LOS_B(1), LOS_B(2)))),180),1);
    
    % Get RCS Value
    rcs_value = rcsData(theta.*10,psi.*10);
    
    % Result
    loop_result(i,:) = [LOS_B(1) LOS_B(1) LOS_B(1) psi theta rcs_value];
    RCS_history(i,:) = rcs_value;
end

% Plot RCS Value
figure(1)
plot(RCS_history)
title("RCS History")
grid on
grid minor

% Plot Azimuth, Elevation, RCS Value
figure(2)
plot(loop_result(:,4:6))
legend("Azimuth","Elevation","RCS Value")
title("Comparison Azimuth, Elevation and RCS")
grid on
grid minor
