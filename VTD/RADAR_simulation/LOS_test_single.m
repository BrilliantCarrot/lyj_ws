function is_visible = LOS_test_single(radar_1, target_1, X, Y, Z)
    % LOS_test_single: Test visibility for a single target point
    % radar_1: [x, y, z] coordinates of the radar
    % target_1: [x, y, z] coordinates of the target
    % X, Y: Meshgrid matrices of terrain coordinates
    % Z: Altitude matrix of the terrain

    radar_x = radar_1(1);
    radar_y = radar_1(2);
    radar_z = radar_1(3);
    target_x = target_1(1);
    target_y = target_1(2);
    target_z = target_1(3);

    % Generate LOS vector
    num_steps = 100;
    los_x = linspace(radar_x, target_x, num_steps);
    los_y = linspace(radar_y, target_y, num_steps);
    los_z = linspace(radar_z, target_z, num_steps);
    
    % Extend LOS beyond the target
    target_ext = target_1 + 10000 * (target_1 - radar_1) / norm(target_1 - radar_1);
    los_x_ext = linspace(target_x, target_ext(1), 100);
    los_y_ext = linspace(target_y, target_ext(2), 100);
    los_z_ext = linspace(target_z, target_ext(3), 100);
    
    % Terrain interpolation
    F = scatteredInterpolant(X(:), Y(:), Z(:), 'linear', 'none');
    is_visible = true;
    
    for k = 1:length(los_x_ext)
        terrain_z = F(los_x_ext(k), los_y_ext(k));
        if ~isnan(terrain_z) && terrain_z > los_z_ext(k)
            is_visible = false;
            break;
        end
    end
    
    % Visualization
    figure;
    clf;
    set(gcf, 'Position', [150, 75, 1200, 750]);
    hold on;
    surf(X, Y, Z, 'EdgeColor', 'None', 'FaceAlpha', 0.5); % Terrain surface
    colormap('jet');
    colorbar;
    view(-20, 85);
    plot3(los_x, los_y, los_z, 'b-', 'LineWidth', 2); % LOS line
    plot3(los_x_ext, los_y_ext, los_z_ext, 'g--', 'LineWidth', 2); % Extended LOS line
    scatter3(radar_x, radar_y, radar_z, 50, 'k', 'filled');
    scatter3(target_x, target_y, target_z, 50, 'r', 'filled');
    if ~is_visible
        obstruct_x = los_x_ext(k);
        obstruct_y = los_y_ext(k);
        obstruct_z = terrain_z;
        scatter3(obstruct_x, obstruct_y, obstruct_z, 50, 'm', 'filled');
    end
    title('LOS Visibility Test');
    xlabel('X [km]');
    ylabel('Y [km]');
    zlabel('Altitude (meters)');
    legend('Terrain', 'LOS Path', 'Extended LOS', 'Radar', 'Target', 'Obstruction');
    view(3);
    grid on;



    % % Radar position
    % radar_x = radar_1(1);
    % radar_y = radar_1(2);
    % radar_z = radar_1(3);
    % 
    % % Target position
    % target_x = target_1(1);
    % target_y = target_1(2);
    % target_z = target_1(3);
    % 
    % % Create the LOS vector from radar to the target
    % num_steps = max(abs(target_x - radar_x), abs(target_y - radar_y));
    % los_x = linspace(radar_x, target_x, num_steps);
    % los_y = linspace(radar_y, target_y, num_steps);
    % los_z = linspace(radar_z, target_z, num_steps);
    % 
    % % Initialize visibility status
    % is_visible = true;
    % 
    % % Check for obstructions along the LOS
    % for k = 2:num_steps-1 % Exclude radar and target points
    %     % Find the nearest cell in the terrain grid
    %     [~, closest_row] = min(abs(Y(:, 1) - los_y(k)));
    %     [~, closest_col] = min(abs(X(1, :) - los_x(k)));
    % 
    %     % Get the altitude of the terrain at the closest cell
    %     terrain_z = Z(closest_row, closest_col);
    % 
    %     % Compare terrain altitude with LOS altitude
    %     if terrain_z > los_z(k)
    %         is_visible = false;
    %         break;
    %     end
    % end
    % 
    % % Visualization
    % figure;
    % clf;
    % set(gcf, 'Position', [150, 75, 1200, 750]); % [left, bottom, width, height]
    % hold on;
    % surf(X, Y, Z, 'EdgeColor', 'None', 'FaceAlpha', 0.5); % Terrain surface
    % colormap('jet');
    % colorbar;
    % view(-20, 85);
    % plot3(los_x, los_y, los_z, 'b-', 'LineWidth', 2); % LOS line
    % scatter3(radar_x, radar_y, radar_z, 50, 'k', 'filled');
    % scatter3(target_x, target_y, target_z, 50, 'r', 'filled');
    % if ~is_visible
    %     obstruct_x = los_x(k);
    %     obstruct_y = los_y(k);
    %     obstruct_z = terrain_z;
    %     scatter3(obstruct_x, obstruct_y, obstruct_z, 50, 'm', 'filled');
    % end
    % title('LOS Visibility Test');
    % xlabel('X [km]');
    % ylabel('Y [km]');
    % zlabel('Altitude (meters)');
    % legend('Terrain', 'LOS Path', 'Radar', 'Target', 'Obstruction');
    % view(3);
    % grid on;
end
