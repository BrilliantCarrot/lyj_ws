

function visualize_PSO_SIR(optimal_path, sir_data, radars, X, Y, Z, start_pos, end_pos)
    figure;
    set(gcf, 'Position', [200, 100, 1000, 750]);
    hold on;
    s = surf(X / 1000, Y / 1000, Z, sir_data, 'EdgeColor', 'none');

	% contour3(X/1000, Y/1000, Z, 10, 'k', 'LineWidth', 1.2); % 등고선 10개
scatter3(radars(:,1)/1000, radars(:,2)/1000, radars(:,3), ...
         150, 'filled', 'MarkerFaceColor', 'yellow', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

    colormap(jet);
    h_start = plot3(start_pos(1) / 1000, start_pos(2) / 1000, start_pos(3), ...
          'ks', 'MarkerSize', 24, 'MarkerFaceColor', 'w');
    h_end = plot3(end_pos(1) / 1000, end_pos(2) / 1000, end_pos(3), ...
          'ks', 'MarkerSize', 24, 'MarkerFaceColor', 'g');
    sir_values = optimal_path(:, 4);
    sir_min = min(sir_values);
    sir_max = max(sir_values);
    alpha(s, 0.8);
    clim([min(sir_data(:)), max(sir_data(:))]);
    xlabel('X Coordinate [km]','FontSize',18);
    ylabel('Y Coordinate [km]','FontSize',18);
    zlabel('Altitude [m]','FontSize',18);
    % title('SIR Distribution and Optimized Path','FontSize',18);
    view(-20, 80);
    grid on;
    h_radar = scatter3(radars(:,1)/1000, radars(:,2)/1000, radars(:,3),100, 'yellow', 'filled');
    h_path = scatter3(optimal_path(:,1)/1000,optimal_path(:,2)/1000,optimal_path(:,3),60, 'k', 'filled');
    legend([s, h_radar, h_path, h_start, h_end],{'SIR distribution', 'Radar Position', ...
        'Optimized Path','Start Position', 'End Position'},'Location', 'best','FontSize',18);
    % 옆에 따로 컬러바 생성
    figure;
    ax2 = axes('Position',[0.1 0.1 0.8 0.8],'Visible','off');
    colormap(ax2, jet);
    caxis(ax2, [sir_min sir_max]);
    cb = colorbar(ax2, 'eastoutside');
    cb.Label.Color = 'k'; % 라벨 글자색
    cb.Color = 'k'; % 눈금 숫자(티크 텍스트) 색상     
    cb.Label.String = 'SIR [dB]';
    cb.Label.FontSize = 12;
end
