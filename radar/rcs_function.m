function rcs_value = rcs_function(theta, phi, rcsData)   
    % 고각, 방위각, rcs 테이블 입력을 받고 테이블상에서 RCS값을 확인, 값을 출력하는 함수
    % 입력: 고각, 방위각, rcs 테이블
    % statement: 고각과 방위각에 따른 보간수행
    % 출력: 입력값에 해당하는 RCS 값

    [numRows, numCols] = size(rcsData);
    [Phi, Theta] = meshgrid(linspace(0,360,numCols), linspace(0,180,numRows));
    % rcs_value = interp2(Phi, Theta, rcsData, phi, theta, "linear");
    phi = mod(phi, 360);
    if phi < 0
        phi = phi + 360;
    end
    theta = mod(theta, 180);
    if theta < 0
        theta = theta + 180;
    end
    rcs_value = interp2(Phi, Theta, rcsData, phi, theta, "cubic");
end
