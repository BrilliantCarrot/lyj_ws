%% RCS History
clear; clc;

% 기체 및 해당 기체의 RCS를 불러오는 변수 M
M = 3;

% Scernario and RCS Data Setting
scenario_list = ["fixed wing scenario", "rotary wing scenario","loitering munition scenario", "cruise missile scenario"];
% RCS_Data_list = ["NEW_F-4_3D", "NEW_AH-1_3D","NEW_UAV_3D", "NEW_Tomahawk_3D"];
RCS_Data_list = ["F-4_3D", "AH-1_3D","UAV_3D", "Tomahawk_3D"];
Output_name_list = ["F-4", "AH-1", "UAV", "Tomahawk"];

scenario_file_folder = scenario_list(M);
RCS_Data = RCS_Data_list(M);
Output_name = Output_name_list(M);

Final_result = [];

for scenario = 1:100

    %Initialize
    clearvars -except Final_result scenario scenario_file_folder RCS_Data Output_name scenario_list RCS_Data_list Output_name_list M;

    % Data Input
    % scernario_filename = sprintf('\\\\Acsl-nas2\\2024\\정명현\\[LIG 넥스원] 분류 알고리즘\\RCS_HISTORY\\LLA\\%s\\%s%d.csv',scenario_file_folder,scenario_file_folder,scenario);
    % rcs_filename = sprintf('\\\\Acsl-nas2\\2024\\정명현\\[LIG 넥스원] 분류 알고리즘\\RCS_HISTORY\\RCS_Data\\%s.csv',RCS_Data);
    scenario_filename = sprintf("C:\\Users\\leeyj\\OneDrive - 인하대학교\\School\\Matlab_Files\\RADAR\\data\\scenario\\%s\\%s%d.csv",scenario_file_folder,scenario_file_folder,scenario);
    rcs_filename = sprintf("C:\\Users\\leeyj\\OneDrive - 인하대학교\\School\\Matlab_Files\\RADAR\\data\\rcs_table\\%s.csv",RCS_Data);
    rawData = readtable(scenario_filename,"VariableNamingRule","preserve");
    rcsData = readmatrix(rcs_filename);
    
    % Aircraft LLA Coordinate
    lla_aircraft = [rawData.Latitude,rawData.Longitude,rawData.Altitude];
    
    % Generate Radar LLA Coordinate
    lat_min = min(lla_aircraft(:,1));
    lon_min = min(lla_aircraft(:,2));
    lat_max = max(lla_aircraft(:,1));
    lon_max = max(lla_aircraft(:,2));
    lla_radar = [(lat_min + lat_max).*0.5,(lon_min + lon_max).*0.5,0];
    
    % Radar LLA Coordinate(Last LLA coordinate of Aircraft)
    % lla_radar = lla_aircraft(length(lla_aircraft),:) ;
    
    % Euler Angle
    euler = [rawData.Roll,rawData.Pitch,rawData.Yaw];
    
    % Ground speed, Vertical speed, Heading
    v_gs = rawData.GS;
    v_vs = rawData.VS;
    heading = deg2rad(rawData.Heading);
    
    % Compute Line of Sight(NED-Frame at body)
    LOS_NED_B = lla2ned(lla_radar,lla_aircraft,'ellipsoid');
    
    % Compute Aircraft Coordinate(NED-Frame at Radar)
    Position_NED_Radar = lla2ned(lla_aircraft,lla_radar,'ellipsoid');
    
    % Compute Velocity of Aircraft(NED-Frame at Radar) 
    Vn_NED_Radar = v_gs.*cos(heading);
    Ve_NED_Radar = v_gs.*sin(heading);
    Vd_NED_Radar = -v_vs;
    
    % Euler Angle Conversion(Radian to degree)
    roll = deg2rad(euler(:,1));
    pitch = deg2rad(euler(:,2));
    yaw = deg2rad(euler(:,3));
    
    % Compute DCM(NED-Frame to B-frame)
    dcm_n2b = angle2dcm(yaw,pitch,roll);
    
    loop_result = [];
    RCS_history = [];
    scen= [];
    time = [];

    phi_values = zeros(1, length(lla_aircraft));
    theta_values = zeros(1, length(lla_aircraft));
    
    for i = 1:(length(lla_aircraft)) 
        % Compute Line of Sight(B-Frame)
        LOS_B = dcm_n2b(:,:,i)*LOS_NED_B(i,:)';
        
        % Calculate Azimuth (0 ~ 360 [deg])
        phi = rad2deg(atan2(LOS_B(2),LOS_B(1)));
    
        % Calculate Elevation (0 ~ 180 [deg])
        theta = rad2deg(atan2(LOS_B(3), hypot(LOS_B(1), LOS_B(2))));

        phi_values(i) = phi;
        theta_values(i) = theta;

        % 반올림을 대신하여 interp2를 이용한 보간 수행
        % 함수를 호출하여 보간
        % Calculate row,column no.
        % rcs_row = round(theta,1)*10 + 901;
        % rcs_column = round(mod(psi,360),1)*10+1;
        
        % Get RCS Value
        % rcs_value = rcsData(rcs_row,rcs_column);
        rcs_value = rcs_function(theta,phi,rcsData);
        
        % Result
        loop_result(i,:) = [LOS_B(1) LOS_B(2) LOS_B(3) atan2(LOS_B(2),LOS_B(1)) atan2(LOS_B(3),hypot(LOS_B(1), LOS_B(2))) phi theta rcs_value];
        RCS_history(i,:) = rcs_value;
        scen(i,:) = scenario;
        time(i,:) = 0.1*i - 0.1;
    end
    
    Result = horzcat(scen,time,Position_NED_Radar,Vn_NED_Radar,Ve_NED_Radar,Vd_NED_Radar,RCS_history);
    % Result = horzcat(phi, theta, scen,time,Position_NED_Radar,Vn_NED_Radar,Ve_NED_Radar,Vd_NED_Radar,RCS_history);
    Final_result = [Final_result; Result;];
    

    % Generate Result Folder
    % folderName = sprintf('\\\\Acsl-nas2\\2024\\정명현\\[LIG 넥스원] 분류 알고리즘\\RCS_HISTORY\\Result\\%s_RCS\\Scenario%d',Output_name,scenario);                                            
    % if ~exist(folderName, 'dir')
    %     mkdir(folderName);
    % end
    % folderName_result = sprintf('\\\\Acsl-nas2\\2024\\정명현\\[LIG 넥스원] 분류 알고리즘\\RCS_HISTORY\\Result\\%s_RCS',Output_name);
    % folderName_result_csv = sprintf('\\\\Acsl-nas2\\2024\\정명현\\[LIG 넥스원] 분류 알고리즘\\RCS_HISTORY\\Result\\%s_RCS\\%s_RCS_Graph.csv',Output_name,Output_name);

    folderName = sprintf("C:\\Users\\leeyj\\OneDrive - 인하대학교\\School\\Matlab_Files\\RADAR\\result\\%s_RCS\\scenario%d",Output_name,scenario);
    if ~exist(folderName,"dir")
        mkdir(folderName);
    end
    folderName_result = sprintf("C:\\Users\\leeyj\\OneDrive - 인하대학교\\School\\Matlab_Files\\RADAR\\result\\%s_RCS",Output_name);
    folderName_result_csv = sprintf("C:\\Users\\leeyj\\OneDrive - 인하대학교\\School\\Matlab_Files\\RADAR\\result\\%s_RCS\\%s_RCS_graph.csv",Output_name, Output_name);

    title1 = sprintf('%s Scenario %d RCS History', Output_name, scenario);
    title2_a= sprintf('%s Scenario %d Azimuth (psi [deg])', Output_name, scenario);
    title2_e= sprintf('%s Scenario %d Elevation (theta [deg]))', Output_name, scenario);
    title2_r= sprintf('%s Scenario %d RCS [m^2])', Output_name, scenario);
    title3 = sprintf('%s Scenario %d NED-coordinate at Radar', Output_name, scenario);
    title4_n = sprintf('%s Scenario %d Vn (North Velocity)', Output_name, scenario);
    title4_e = sprintf('%s Scenario %d Ve (East Velocity)', Output_name, scenario);
    title4_d = sprintf('%s Scenario %d Vd (Down Velocity)', Output_name, scenario);
    title5 = sprintf('%s RCS Value', Output_name);

    % Plot RCS Value and Save
    figure(1)
    plot(RCS_history)
    xlabel('Time');
    ylabel('RCS [m^2]');
    grid on
    grid minor
    title(title1)
    savefig(gcf, fullfile(folderName, 'RCS_History.fig'));

    
    % Plot Azimuth, Elevation, RCS Value and Save
    figure(2)
    plot(loop_result(:,6:8))
    % Azimuth (psi [deg]) Plot
    subplot(3, 1, 1); 
    plot(loop_result(:,6), 'LineWidth', 2);
    title(title2_a);
    xlabel('Time');
    ylabel('Azimuth [deg]');
    grid on;
    grid minor;
    % Elevation (theta [deg]) Plot
    subplot(3, 1, 2);
    plot(loop_result(:,7), 'LineWidth', 2);
    title(title2_e);
    xlabel('Time');
    ylabel('Elevation [deg]');
    grid on;
    grid minor
    % RCS Value Plot
    subplot(3, 1, 3);
    plot(loop_result(:,8), 'LineWidth', 2);
    title(title2_r);
    xlabel('Time');
    ylabel('RCS [m^2]');
    grid on;
    grid minor;
    savefig(gcf, fullfile(folderName, 'Azimuth_Elevation_RCS.fig'));

    % Plot Aircraft Position NED-Frame at Radar and Save
    figure(3)
    plot3(Position_NED_Radar(:,1),Position_NED_Radar(:,2),Position_NED_Radar(:,3))
    set(gca, 'ZDir', 'reverse');
    xlabel('N [m]');
    ylabel('E [m]');
    zlabel('D [m]');
    grid on
    title(title3)
    savefig(gcf, fullfile(folderName, 'Aircraft_Position_NED.fig'));

    % Plot Aircraft Velocity NED at Radar and Save
    figure(4)
    x = 0:1:length(Vn_NED_Radar)-1;
    % Vn (North Velocity) 그래프
    subplot(3, 1, 1);
    plot(x, Vn_NED_Radar(:,1), 'LineWidth', 2);
    title(title4_n);
    xlabel('Time');
    ylabel('Velocity [m/s]');
    grid on;
    % Ve (East Velocity) 그래프
    subplot(3, 1, 2);
    plot(x, Ve_NED_Radar(:,1), 'LineWidth', 2);
    title(title4_e);
    xlabel('Time');
    ylabel('Velocity [m/s]');
    grid on;
    % Vd (Down Velocity) 그래프
    subplot(3, 1, 3);
    plot(x, Vd_NED_Radar(:,1), 'LineWidth', 2);
    title(title4_d);
    xlabel('Time');
    ylabel('Velocity [m/s]');
    grid on;
    title(title4_d)
    savefig(gcf, fullfile(folderName, 'Aircraft_Velocity_NED.fig'));

end

% Plot rcsData
figure(5)
imagesc(rcsData)
colorbar
title(title5);
savefig(gcf, fullfile(folderName_result, 'RCS_Value.fig'));                                                

%Csv output
T = array2table(Final_result, 'VariableNames', {'Scenario','Time','N', 'E', 'D','Vn','Ve','Vd','RCS'});
writetable(T, folderName_result_csv);                                                                     