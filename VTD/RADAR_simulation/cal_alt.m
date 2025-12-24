function z = cal_alt(x,y,X,Y,Z)
    % Ensure x and y are within the range of X and Y
    if x < min(X(:)) || x > max(X(:)) || y < min(Y(:)) || y > max(Y(:))
        z = NaN; % Return NaN if x or y is out of range
        return;
    end

    % Find nearest indices
    [idx1, idx2] = find(X(1,:) - x < 0);
    [idx3, idx4] = find(Y(:,1) - y < 0);

    % Handle empty results
    if isempty(idx1) || isempty(idx3)
        z = NaN; % Return NaN if indices cannot be determined
        return;
    end

    ix = idx3(end);
    iy = idx2(end);

    % Determine grid points for interpolation
    ix11 = ix; iy11 = iy;
    ix12 = ix; iy12 = iy + 1;
    ix21 = ix + 1; iy21 = iy;
    ix22 = ix + 1; iy22 = iy + 1;

    % Ensure indices do not exceed matrix dimensions
    ix21 = min(ix21, size(X, 1));
    ix22 = min(ix22, size(X, 1));
    iy12 = min(iy12, size(X, 2));
    iy22 = min(iy22, size(X, 2));

    % Interpolate Z values
    x11 = X(ix11, iy11); y11 = Y(ix11, iy11); z11 = Z(ix11, iy11);
    x12 = X(ix12, iy12); y12 = Y(ix12, iy12); z12 = Z(ix12, iy12);
    x21 = X(ix21, iy21); y21 = Y(ix21, iy21); z21 = Z(ix21, iy21);
    x22 = X(ix22, iy22); y22 = Y(ix22, iy22); z22 = Z(ix22, iy22);

    % Calculate distances
    r11 = norm([x11 - x, y11 - y]);
    r12 = norm([x12 - x, y12 - y]);
    r21 = norm([x21 - x, y21 - y]);
    r22 = norm([x22 - x, y22 - y]);

    % Weighted average of Z values
    z = (r11 * z11 + r12 * z12 + r21 * z21 + r22 * z22) / (r11 + r12 + r21 + r22);
end