function visualize_PSO_gray(optimal_path, ~ , radar_pos, X, Y, Z, start_pos, end_pos)
    % optimal_path: PSO 알고리즘 결과로 생성된 최적 경로
    % sir_data: PSO 알고리즘에서 각 단계별로 계산된 SIR 분포 데이터
    % radar_pos: 레이더 위치
    % X, Y, Z: 지형 데이터
    figure;
    set(gcf, 'Position', [200, 100, 1000, 750]);
    hold on;
    s = surf(X / 1000, Y / 1000, Z, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
    colormap(gray);
    % contour3(X/1000, Y/1000, Z, 10, 'k', 'LineWidth', 1.2); % 등고선 10개
    % colorbar;
    xlabel('X Coordinate [km]','FontSize',18);
    ylabel('Y Coordinate [km]','FontSize',18);
    zlabel('Altitude [m]','FontSize',18);
    % title('Path Minimizing Detectability Over Flat Terrain','FontSize',18);
    view(-20, 80);
    grid on;
    h_radar = scatter3(radar_pos(:,1)/1000, radar_pos(:,2)/1000, radar_pos(:,3),100, 'yellow',...
        'filled','MarkerEdgeColor','k','LineWidth',1.5);
    h_start = plot3(start_pos(1) / 1000, start_pos(2) / 1000, start_pos(3)+20, ...
          'ks', 'MarkerSize', 24, 'MarkerFaceColor', 'w');
    h_end = plot3(end_pos(1) / 1000, end_pos(2) / 1000, end_pos(3)+15, ...
          'ks', 'MarkerSize', 24, 'MarkerFaceColor', 'g');

    sir_values = optimal_path(:, 4);
    sir_min = min(sir_values);
    sir_max = max(sir_values);
    cmap = jet(256);
    for t = 1:length(optimal_path)
        x = optimal_path(t, 1) / 1000;
        y = optimal_path(t, 2) / 1000;
        z = optimal_path(t, 3);
        sir = optimal_path(t, 4);
        idx = round((sir - sir_min) / (sir_max - sir_min) * 255) + 1;
        idx = max(1, min(256, idx)); % 범위 보정
        color = cmap(idx, :);
        h_path = plot3(x, y, z, 'o', 'MarkerSize', 8,'MarkerFaceColor', color, 'MarkerEdgeColor', 'k');
    end
    legend([s, h_radar, h_path, h_start, h_end],{'Terrain', 'Radar Position', ...
    'Optimized Path','Start Position', 'End Position'},'Location', 'best','FontSize',18);
    figure;
    ax2 = axes('Position',[0.1 0.1 0.8 0.8], ...  % 화면 꽉 채우되, margin 조절
           'Visible','off');                 % axes, tick, grid 모두 숨김
    colormap(ax2, jet);
    caxis(ax2, [sir_min sir_max]);
    cb = colorbar(ax2, 'eastoutside');
    cb.Label.String = 'SNR [dB]';
    cb.Label.FontSize = 12;

end
