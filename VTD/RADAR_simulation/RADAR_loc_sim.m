function sig_matrix = RADAR_loc_sim(radars, X, Y, Z, RADAR)
    % radar_pos: [x, y, z] 레이더 위치
    % X, Y, Z: 지형 데이터 (격자 행렬)
    % RADAR: 레이더 파라미터 구조체
    % sig_matrix: 각 지형 셀에 대한 SIR 값을 저장하는 행렬
    lambda = RADAR.lambda;
    Pt = RADAR.Pt;
    tau = RADAR.tau;
    G = RADAR.G;
    Ts = RADAR.Ts;
    L = RADAR.L;
    sigma_0 = RADAR.sigma_0;
    theta_A = RADAR.theta_A;
    theta_E = RADAR.theta_E;
    SL_rms = RADAR.SL_rms;
    R_e = RADAR.R_e;
    c = RADAR.c;
    prf = RADAR.prf;
    Du = RADAR.Du;
    rcs_table = RADAR.RCS1;
    [rows, cols] = size(Z);
    sig_matrix = zeros(rows, cols);
    num_radars = size(radars, 1);
	for r = 1:num_radars
		radar_pos = radars(r,:);
		for i = 1:rows
			for j = 1:cols
				target_pos = double([X(i, j), Y(i, j), Z(i, j)]);
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
				rcs = 10^(rcs / 10);  % dB를 선형 스케일로 변환
				Fecl = eclipsingfactor(Range, Du, prf);
				SNR = radareqsnr(lambda, Range, Pt, tau, 'Gain', G, 'Ts', Ts, 'RCS', rcs, 'CustomFactor', Fecl, 'Loss', L);
				h_r = double(radar_pos(3));  % 레이더 고도
				h_t = double(Z(i, j) + 100);  % 목표물 고도, 원하는 값을 더하고 빼도됨(비행체 고도 조절)
                % h_t = double(100);
				Range = double(Range);  % Range도 double로 변환
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
				sigma_clutter = (sigma_MBc + sigma_SLc); % 단순화한 지형의 경우 곡률 보정 불필요
				% sigma_clutter = (sigma_MBc + sigma_SLc) ./ (1 + (Range ./ R_h).^4);
				% sigma_clutter = (sigma_MBc + sigma_SLc) ./ (1 + (Range ./ R_h).^2);
				SCR = rcs / sigma_clutter;
				SIR = 1 / ((1 / SNR) + (1 / SCR));
				SIR_dB = 10 * log10(SIR);
				SNR_dB = 10*log10(SNR);
				sig_matrix(i, j) = max(sig_matrix(i,j), SIR_dB);
			end
		end
	end
end