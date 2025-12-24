function sig = RADAR_Module(RADAR,PosN,lambda_num)

% 10dBsm = 10m²,  x dBsm =(10^(x/10))m^2
% SNR : Signal to Noise Ratio is standard measure of a radar's ability to
% detect a given target at range from radar
% SNR = (P_t[W] * G^2 * lambda^2[m^2] * RCS[m^2]) / (4pi^2 * R^4[m^4] * N(=k * T_s * B_n)[W] * L)
% Using Matlab radar toolbox
% Reference Radar : S-band(RL-2000/GCI), X-band(Scanter 4000)


   % for lambda num = 1, 2GHz
       if lambda_num == 1
            rcs_table = RADAR.RCS1;
            pitch_array = RADAR.theta(1,:) * pi/180;
            yaw_array = RADAR.psi(:,1) * pi/180;
            sig = 0;
            lambda = freq2wavelen(2*10^9);          % [m] wavelength
            Pt = 14000;                             % [W] peak power
            tau = 0.00009;                          % [s] pulse width
            G = 34;                                 % [dBi] antenna gain
            Ts = 290;                               % [K] System temp 
            L = 8.17;                                  % [dB] Loss
            prf = 1000;                             % [Hz] Pulse repetition frequency 
    % for lambda num = 2, 8GHz
            elseif lambda_num == 2
            rcs_table = RADAR.RCS2;
            pitch_array = RADAR.theta(1,:) * pi/180;
            yaw_array = RADAR.psi(:,1) * pi/180;
            sig = 0;
            lambda = freq2wavelen(8*10^9);          % [m] wavelength
            Pt = 6000;                              % [W] peak power
            tau = 0.0001;                           % [s] pulse width
            G = 39;                                 % [dBi] antenna gain
            Ts = 290;                               % [K] System temp 
            L = 0;                                  % [dB] Loss
            prf = 2200;                             % [Hz] Pulse repetition frequency 

        end

%         for i = 1:RADAR.N_Radar
            RelPos = - RADAR.RadarPos(lambda_num,:) + PosN;
            Range = norm(RelPos);
            los_pitch = atan2(-RelPos(3),norm(RelPos(1:2)));
            los_yaw = atan2(RelPos(2),RelPos(1));
            pitch = Angle_trim(los_pitch);  
            yaw = Angle_trim(los_yaw);
            p_idx = Find_Index(pitch_array,length(pitch_array),pitch);
            y_idx = Find_Index(yaw_array,length(yaw_array),yaw);
            p_lower = rcs_table(:,p_idx);
            p_upper = rcs_table(:,p_idx+1);
            p_rcs = p_lower + (pitch-pitch_array(p_idx))*(p_upper-p_lower)/(pitch_array(p_idx+1)-(pitch_array(p_idx)));
            y_lower = p_rcs(y_idx,:);
            y_upper = p_rcs(y_idx+1,:);
            rcs = y_lower + (yaw-yaw_array(y_idx))*(y_upper-y_lower)/(yaw_array(y_idx+1)-(yaw_array(y_idx)));
%             rcs_min = min(min(rcs_table));

            rcs =   10^(rcs/10);
            Du = tau * prf;
            Fecl = eclipsingfactor(Range, Du, prf);
            sig = radareqsnr(lambda,Range,Pt,tau,'Gain',G,'Ts',Ts,'RCS',rcs,'CustomFactor',Fecl,'Loss',L);

%         end
       
%      

end

