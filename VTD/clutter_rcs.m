function [sigmaC, CNR] = clutter_rcs(sigma0, thetaE, thetaA, SL, range, hr, ht, ...
                                     pt, f0, b, t0, f, l, ant_id)
    % This function calculates the clutter RCS and the CNR for a ground-based radar.
    clight = 3e8; % speed of light in meters per second
    lambda = clight / f0;
    thetaA_deg = thetaA;
    thetaE_deg = thetaE;
    thetaA = thetaA_deg * pi / 180; % antenna azimuth beamwidth in radians
    thetaE = thetaE_deg * pi / 180; % antenna elevation beamwidth in radians
    re = 6371000; % earth radius in meters
    rh = sqrt(8.0 * hr * re / 3); % range to horizon in meters
    SLv = 10^(SL / 10); % radar rms sidelobes in volts
    sigma0v = 10^(sigma0 / 10); % clutter backscatter coefficient
    tau = 1 / b; % pulsewidth
    deltar = clight * tau / 2; % range resolution for unmodulated pulse

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    range_m = 1000 * range; % range in meters
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    thetar = asin(hr ./ range_m);
    thetae = asin((ht - hr) ./ range_m);
    propag_atten = 1 + (range_m ./ rh).^4; % propagation attenuation due to round earth
    Rg = range_m .* cos(thetar);
    deltaRg = deltar * cos(thetar);
    theta_sum = thetae + thetar;

    % use sinc^2 antenna pattern when ant_id=1, use Gaussian antenna pattern when ant_id=2
    if ant_id == 1 % use sinc^2 antenna pattern
        ant_arg = (2.78 * theta_sum) ./ (pi * thetaE);
        gain = sinc(ant_arg).^2;
    else
        gain = exp(-2.776 * (theta_sum ./ thetaE).^2);
    end

    % compute sigmac
    sigmac = (sigma0v .* Rg .* deltaRg) .* (pi * SLv * SLv + thetaA .* gain.^2) ./ propag_atten;
    sigmaC = 10 * log10(sigmac);

    % Plot or display sigmaC
    if size(range, 2) == 1
        fprintf('Sigma_Clutter='); disp(sigmaC)
    else
        figure(1)
        plot(range, sigmaC)
        grid on
        xlabel('Slant Range in Km')
        ylabel('Clutter RCS in dBsm')
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculate CNR
    pt = pt * 1000;
    g = 26000 / (thetaA_deg * thetaE_deg); % antenna gain
    F = 10^(f / 10); % noise figure
    Lt = 10^(l / 10); % total radar losses
    k = 1.38e-23; % Boltzmann's constant
    T0 = t0; % noise temperature

    argnumC = 10 * log10(pt * g * g * lambda * lambda * tau .* sigmac);
    argdem = 10 * log10(((4 * pi)^3) * k * T0 * Lt * F .* (range_m).^4);
    CNR = argnumC - argdem;

    % Plot or display CNR
    if size(range, 2) == 1
        fprintf('Clutter_to_Noise_ratio='); disp(CNR)
    else
        figure(2)
        plot(range, CNR, 'r')
        grid on
        xlabel('Slant Range in Km')
        ylabel('CNR in dB')
    end
end