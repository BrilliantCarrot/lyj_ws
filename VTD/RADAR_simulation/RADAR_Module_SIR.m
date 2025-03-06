function [sig1,sigma_MBc, sigma_SLc,sigma_clutter2,SNR,SCR2,SIR_dB_air,Range] = RADAR_Module_SIR(block_check, RADAR,PosN,lambda_num,X,Y,Z)    
    % 레이더 파라미터 부분
    rcs_table = RADAR.RCS1;
    pitch_array = RADAR.theta(1,:) * pi/180;
    yaw_array = RADAR.psi(:,1) * pi/180;
    % sig = 0;
    lambda = freq2wavelen(2*10^9);          % [m] wavelength
    Pt = 14000;                             % [W] peak power
    tau = 0.00009;                          % [s] pulse width
    G = 34;                                 % [dBi] antenna gain
    Ts = 290;                               % [K] System temp 
    L = 8.17;                                  % [dB] Loss
    prf = 1000;                             % [Hz] Pulse repetition frequency 
    
    % 위치 계산
    % for i = 1:RADAR.N_Radar
    RelPos = - RADAR.RadarPos(lambda_num,:) + PosN;
    Range = norm(RelPos);   % 레이더와 기체 간 거리(Slant Range)
    los_pitch = atan2(-RelPos(3),norm(RelPos(1:2)));
    los_yaw = atan2(RelPos(2),RelPos(1));   % los_yaw: LOS 벡터의 방위각

    % RCS 계산
    pitch = Angle_trim(los_pitch);          % 목표물과 레이더 간 실제 고각
    yaw = Angle_trim(los_yaw);              
    % p_idx: RCS 테이블에서 고각(pitch)에 해당하는 데이터를 참조하기 위한 인덱스 값
    p_idx = Find_Index(pitch_array,length(pitch_array),pitch);
    y_idx = Find_Index(yaw_array,length(yaw_array),yaw);
    p_lower = rcs_table(:,p_idx);
    p_upper = rcs_table(:,p_idx+1);
    p_rcs = p_lower + (pitch-pitch_array(p_idx))*(p_upper-p_lower)/(pitch_array(p_idx+1)-(pitch_array(p_idx)));
    y_lower = p_rcs(y_idx,:);
    y_upper = p_rcs(y_idx+1,:);
    rcs = y_lower + (yaw-yaw_array(y_idx))*(y_upper-y_lower)/(yaw_array(y_idx+1)-(yaw_array(y_idx)));   % dB
    % rcs_min = min(min(rcs_table));
    
    rcs =   10^(rcs/10);    % log 스케일 rcs
    Du = tau * prf;
    Fecl = eclipsingfactor(Range, Du, prf);

    % STC Factor
    Rstc = 50e3;    % STC Cutoff Range (m)
    Xstc = 4;       % STC Exponent
    Fstc = stcfactor(Range, Rstc, Xstc);

    % dB 기준 SNR
    SNR = radareqsnr(lambda,Range,Pt,tau,'Gain',G,'Ts',Ts,'RCS',rcs,'CustomFactor',Fecl+Fstc,'Loss',L);

    c = 3e8;                      % 전파 속도 (m/s)
    sigma_0 = 10^(-20/10);        % 클러터 산란(반사)계수 (선형(log) 스케일), -20 dB(Flatland)
    theta_A = deg2rad(1);         % 방위각 빔폭 (rad)
    theta_E = deg2rad(2);         % 고각 빔폭 (rad)
    SL_rms = 10^(-20.10);         % 사이드로브의 RMS 수준(선형(log) 스케일), -20 dBc = 3e8
    h_r = RADAR.RadarPos(1, 3);   % RADAR.RadarPos의 세 번째 요소가 레이더 고도
    h_t = PosN(3);                % PosN의 세 번째 요소가 목표물의 고도
    R_e = 6.371e6;                % 지구 반지름 (m)
    theta_r = asin(min(1, max(-1, h_r ./ Range)));          % 지표에서 레이더 높이까지의 각도
    theta_e = asin(min(1, max(-1, (h_t - h_r) ./ Range)));  % 레이더 높이에서 기체까지의 각도
    Rg = Range .* cos(theta_r);                             % Slant Range의 지표 투영
    delta_R = c * tau / 2;                                  % Slant Range의 거리 해상도
    delta_Rg = delta_R * cos(theta_r);                      % 지표면 투영 거리 해상도
    theta_sum = theta_e + theta_r;                      
    G_theta = exp(-2.776 * (theta_sum ./ theta_E).^2);  % 고각 및 방위각 두개에 대한 안테나 이득
    % 메인빔 클러터 면적 및 RCS 계산
    A_MBc = delta_Rg .* Rg * theta_A;
    sigma_MBc = sigma_0 .* A_MBc .* G_theta.^2;
    % 사이드로브 클러터 면적 및 RCS 계산
    A_SLc = delta_Rg .* pi .* Rg;
    sigma_SLc = sigma_0 .* A_SLc .* SL_rms.^2;
    % 레이다 탐지 범위의 성분 중 지평선 축의 거리
    R_h = sqrt((8 * R_e * h_r)/3);

    % 클러터 RCS 계산
    sigma_TOTc = (sigma_MBc + sigma_SLc) ./ (1 + (Range / R_h).^4);
    sigma_clutter1 = sigma_TOTc;        % 지형 클러터의 RCS (m^2)

    % SCR = (Pt * G^2 * rcs * lambda^2) ./ (Pt * G^2 * sigma_clutter * lambda^2);

    % SCR, SNR, SIR 모두 무차원

    % 기체 뒤 배경이 존재하지 않으면 Ground Based Radar의 클러터 rcs를 계산
    SCR1 = rcs./sigma_clutter1;
    SIR1 = 1./((1./SNR)+(1./SCR1));           % 클러터의 영향이 고려된 목표물의 SNR 값을 SIR(SCNR)로 정의
    SIR_dB_land = 10 * log10(SIR1);           % dB로 표현된 최종 SIR 값을 출력
    % 기체 뒤 배경이 존재하면 Airbone Radar의 클러터 rcs를 계산        
    propag_atten = (1 + (Range / R_h).^4);              % 지구 곡률로 인한 전파 감쇠
    sigma_clutter2 = (sigma_0 .* Rg .* delta_Rg) .* (pi * SL_rms * SL_rms + theta_A .* G_theta.^2) ./ propag_atten;
    SCR2 = rcs./sigma_clutter2;
    SIR2 = 1./((1./SNR)+(1./SCR2));
    SIR_dB_air = 10 * log10(SIR2);
    % 만약 뒤가 막혀있다면 Airbone Radar 경우
    if block_check == true
        sig1 = SIR_dB_land;
    % 그렇지 않고 배경이 뚫려있다면 Ground Based Radar의 경우
    else
        % sig1 = SIR_dB_land;
        sig1 = SNR;
    end
end