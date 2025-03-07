function visualize_PSO_SIR_2(optimal_path, sir_data, sir_values, visibility_values, radar_pos, X, Y, Z)
    % optimal_path: PSO 알고리즘 결과로 생성된 최적 경로
    % sir_data: PSO 알고리즘에서 각 단계별로 계산된 SIR 분포 데이터
    % radar_pos: 레이더 위치
    % X, Y, Z: 지형 데이터
    figure;
    set(gcf, 'Position', [200, 100, 1000, 750]);
    hold on;
    s = surf(X / 1000, Y / 1000, Z, 'EdgeColor', 'k', 'LineWidth', 1, 'FaceAlpha', 0.5);
    colormap(jet);
    colorbar;
    clim([min(cellfun(@(x) min(x(:)), sir_data)), max(cellfun(@(x) max(x(:)), sir_data))]);
    xlabel('X [km]');
    ylabel('Y [km]');
    zlabel('Altitude [m]');
    title('Optimized Path and SIR Distribution');
    view(-20, 80);
    grid on;
    plot3(radar_pos(1) / 1000, radar_pos(2) / 1000, radar_pos(3), ...
          'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    path_plot = plot3(optimal_path(1, 1) / 1000, optimal_path(1, 2) / 1000, optimal_path(1, 3), ...
                      'r-', 'LineWidth', 2);
    radius_plot = plot3([], [], [], 'c--', 'LineWidth', 1);

    % 텍스트 핸들 배열 (이전 텍스트 삭제하며 최신 업데이트 부분만 표시)
    text_handle = gobjects(1);

    pause_time = 1;
    pause(3);
    for t = 1:length(optimal_path)
        set(path_plot, 'XData', optimal_path(1:t, 1) / 1000, ...
                       'YData', optimal_path(1:t, 2) / 1000, ...
                       'ZData', optimal_path(1:t, 3));
        sir_matrix = sir_data{t};
        set(s, 'CData', sir_matrix);
        current_pos = optimal_path(t, :);
        search_radius = 10000; % 탐색 반경 (반경이 동적으로 변할 경우 업데이트 가능)
        [circle_x, circle_y] = generate_circle(current_pos(1), current_pos(2), search_radius, X, Y);
        set(radius_plot, 'XData', circle_x / 1000, 'YData', circle_y / 1000,...
            'ZData', ones(size(circle_x)) * current_pos(3));

        % 텍스트 표시 (SIR 값과 가시성 여부 표시)
        if visibility_values(t) == 1
            visibility_str = 'Visible';
        else
            visibility_str = 'Hidden';
        end
        % 기존 텍스트 삭제
        if isgraphics(text_handle)
            delete(text_handle);
        end
        % 현재 포인트에 텍스트 추가(흰색 배경 추가)
        text_handle = text(current_pos(1) / 1000, current_pos(2) / 1000, current_pos(3)+100, ...
            sprintf('SIR: %.1f dB\n%s', sir_values(t), visibility_str), ...
            'FontSize', 14, 'Color', 'k', 'HorizontalAlignment', 'left',...
            'BackgroundColor', 'w', 'EdgeColor', 'k', 'Margin', 3);

        drawnow;
        pause(pause_time);
    end
    legend('Terrain', 'Radar Position', 'Optimized Path', 'Search Radius', 'Location', 'Best');
end
% 탐색 반경을 나타내는 원 생성 함수
function [circle_x, circle_y] = generate_circle(center_x, center_y, radius, X, Y)
    theta = linspace(0, 2 * pi, 100);
    circle_x = center_x + radius * cos(theta);
    circle_y = center_y + radius * sin(theta);
    % X, Y 범위 안에서만 원이 생성되도록 제한
    circle_x = max(min(circle_x, max(X(:))), min(X(:)));
    circle_y = max(min(circle_y, max(Y(:))), min(Y(:)));
end