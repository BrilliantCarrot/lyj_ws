function visualize_PSO_SIR(optimal_path, sir_data, radars, X, Y, Z, start_pos, end_pos)
    figure;
    set(gcf, 'Position', [200, 100, 1000, 750]);
    hold on;
    s = surf(X / 1000, Y / 1000, Z, sir_data, 'EdgeColor', 'none');
    colormap(jet);
    h_start = plot3(start_pos(1) / 1000, start_pos(2) / 1000, start_pos(3), ...
          'ks', 'MarkerSize', 15, 'MarkerFaceColor', 'w');
    h_end = plot3(end_pos(1) / 1000, end_pos(2) / 1000, end_pos(3), ...
          'ks', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
    sir_values = optimal_path(:, 4);
    sir_min = min(sir_values);
    sir_max = max(sir_values);
    alpha(s, 0.8);
    clim([min(sir_data(:)), max(sir_data(:))]);
    xlabel('X [km]');
    ylabel('Y [km]');
    zlabel('Altitude [m]');
    title('Optimized Path and SIR Distribution');
    view(-20, 80);
    grid on;
    h_radar = scatter3(radars(:,1)/1000, radars(:,2)/1000, radars(:,3),100, 'yellow', 'filled');
    h_path = scatter3(optimal_path(:,1)/1000,optimal_path(:,2)/1000,optimal_path(:,3),60, 'k', 'filled');
    legend([s, h_radar, h_path, h_start, h_end],{'SIR distribution', 'Radar Position', ...
        'Optimized Path','Start Position', 'End Position'},'Location', 'best');

    figure;
    ax2 = axes('Position',[0.1 0.1 0.8 0.8], ...  % 화면 꽉 채우되, margin 조절
           'Visible','off');                 % axes, tick, grid 모두 숨김
    colormap(ax2, jet);
    caxis(ax2, [sir_min sir_max]);
    cb = colorbar(ax2, 'eastoutside');
    cb.Label.Color = 'k';         % 라벨 글자색 흰색으로
    cb.Color = 'k';               % 눈금 숫자(티크 텍스트) 색상도 흰색으로      
    cb.Label.String = 'SIR (dB)';
    cb.Label.FontSize = 12;

end
