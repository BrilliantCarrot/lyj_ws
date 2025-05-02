function visualize_PSO_gray(optimal_path, sir_data, radar_pos, X, Y, Z)
    % optimal_path: PSO 알고리즘 결과로 생성된 최적 경로
    % sir_data: PSO 알고리즘에서 각 단계별로 계산된 SIR 분포 데이터
    % radar_pos: 레이더 위치
    % X, Y, Z: 지형 데이터
    figure;
    set(gcf, 'Position', [200, 100, 1000, 750]);
    hold on;
    s = surf(X / 1000, Y / 1000, Z, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
    colormap(gray);
    % colorbar;

    xlabel('X [km]');
    ylabel('Y [km]');
    zlabel('Altitude [m]');
    title('Path Minimizing Detectability');
    view(-20, 80);
    grid on;
    % 레이더 표시
    plot3(radar_pos(1) / 1000, radar_pos(2) / 1000, radar_pos(3), ...
          'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    % path_plot = plot3(optimal_path(1, 1) / 1000, optimal_path(1, 2) / 1000, optimal_path(1, 3), ...
    %                   'r-', 'LineWidth', 2);
    % radius_plot = plot3([], [], [], 'c--', 'LineWidth', 1);
    % pause_time = 0.5;


    %3. 경로점의 마커를 SIR 크기로 컬러 표시
    % pause_time = 0.1;
    sir_values = optimal_path(:, 4);
    sir_min = min(sir_values);
    sir_max = max(sir_values);
    cmap = jet(256);

    for t = 1:length(optimal_path)
        x = optimal_path(t, 1) / 1000;
        y = optimal_path(t, 2) / 1000;
        z = optimal_path(t, 3);
        sir = optimal_path(t, 4);

        % 색상 인덱스 계산
        idx = round((sir - sir_min) / (sir_max - sir_min) * 255) + 1;
        idx = max(1, min(256, idx)); % 범위 보정
        color = cmap(idx, :);

        % 마커 표시
        plot3(x, y, z, 'o', 'MarkerSize', 8, ...
              'MarkerFaceColor', color, 'MarkerEdgeColor', 'k');

        drawnow;
        % pause(pause_time);
    end
    legend('Terrain', 'Radar Position', 'Path Points', 'Location', 'Best');

    figure;
    ax2 = axes('Position',[0.1 0.1 0.8 0.8], ...  % 화면 꽉 채우되, margin 조절
           'Visible','off');                 % axes, tick, grid 모두 숨김
    colormap(ax2, jet);
    caxis(ax2, [sir_min sir_max]);
    cb = colorbar(ax2, 'eastoutside');
    cb.Label.String = 'SIR (dB)';
    cb.Label.FontSize = 12;

end
