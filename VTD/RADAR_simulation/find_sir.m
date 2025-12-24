function sig = find_sir(radar_pos, target_pos, RADAR)
    % radar_pos: 레이더 위치 좌표
    % target_pos: 목표물(각각의 지형)좌표
    % RADAR: 레이더 구조체
    % 레이더 파라미터 설정
    lambda = freq2wavelen(2 * 10^9); % 기본 2GHz 파라미터
    Pt = 14000;  % [W] Peak Power
    tau = 0.00009;  % [s] Pulse Width
    G = 34;  % [dBi] Antenna Gain
    Ts = 290;  % [K] System Temperature
    L = 8.17;  % [dB] Loss
    sigma_0 = 10^(-20/10);  % Clutter Scattering Coefficient
    theta_A = deg2rad(1);  % Azimuth Beamwidth
    theta_E = deg2rad(2);  % Elevation Beamwidth
    SL_rms = 10^(-20.10);  % RMS Sidelobe Level
    R_e = 6.371e6;  % Earth Radius (m)
    c = 3e8;  % Speed of Light (m/s)
    prf = 1000; % [Hz] Pulse repetition frequency
    Du = tau * prf;
    rcs_table = RADAR.RCS1;
    % 거리 및 LOS, RCS, SNR 계산
    RelPos = target_pos - radar_pos;  % 레이더와 목표물 간 상대 위치
    Range = norm(RelPos);  % 거리 (Slant Range)
    los_pitch = atan2(-RelPos(3), norm(RelPos(1:2)));
    los_yaw = atan2(RelPos(2), RelPos(1));
    pitch = Angle_trim(los_pitch);
    yaw = Angle_trim(los_yaw);
    pitch_array = RADAR.theta(1, :) * pi/180;
    yaw_array = RADAR.psi(:, 1) * pi/180;
    p_idx = Find_Index(pitch_array, length(pitch_array), pitch);
    y_idx = Find_Index(yaw_array, length(yaw_array), yaw);
    p_lower = rcs_table(:, p_idx);
    p_upper = rcs_table(:, p_idx + 1);
    p_rcs = p_lower + (pitch - pitch_array(p_idx)) * (p_upper - p_lower) / (pitch_array(p_idx + 1) - pitch_array(p_idx));
    y_lower = p_rcs(y_idx, :);
    y_upper = p_rcs(y_idx + 1, :);
    rcs = y_lower + (yaw - yaw_array(y_idx)) * (y_upper - y_lower) / (yaw_array(y_idx + 1) - yaw_array(y_idx));
    rcs = 10^(rcs / 10);  % dB to linear scale
    Fecl = eclipsingfactor(Range, Du, prf);
    SNR = radareqsnr(lambda, Range, Pt, tau, 'Gain', G, 'Ts', Ts, 'RCS', rcs, 'CustomFactor', Fecl, 'Loss', L);
    % 클러터 RCS 계산
    h_r = radar_pos(3);  % 레이더 고도
    h_t = target_pos(3); % 목표물 고도
    theta_r = asin(min(1, max(-1, h_r / Range)));
    theta_e = asin(min(1, max(-1, (h_t - h_r) / Range)));
    Rg = Range * cos(theta_r);  % Ground Range
    delta_R = c * tau / 2;
    delta_Rg = delta_R * cos(theta_r);
    theta_sum = theta_e + theta_r;
    G_theta = exp(-2.776 * (theta_sum / theta_E)^2);
    A_MBc = delta_Rg * Rg * theta_A;
    sigma_MBc = sigma_0 * A_MBc * G_theta^2;
    A_SLc = delta_Rg * pi * Rg;
    sigma_SLc = sigma_0 * A_SLc * SL_rms^2;
    R_h = sqrt((8 * R_e * h_r) / 3);
    sigma_clutter = (sigma_MBc + sigma_SLc) / (1 + (Range / R_h)^4);
    % SCR 및 SIR 계산
    SCR = rcs / sigma_clutter;
    SIR = 1 / ((1 / SNR) + (1 / SCR));
    SIR_dB = 10 * log10(SIR);
    sig = SIR_dB;
end