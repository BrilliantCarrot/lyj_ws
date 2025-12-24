clear
close all
dt = 0.05;
t = 0:dt:20;

Nsamples = length(t);

Xsaved = zeros(Nsamples,3);
Zsaved = zeros(Nsamples,1);

for k = 1:Nsamples
    r = GetRadar(dt);

    [pos,vel,alt] = RadarEKF(r,dt);

    Xsaved(k,:) = [pos vel alt];
    Zsaved(k) = r;
end


PosSaved = Xsaved(:,1);
VelSaved = Xsaved(:,2);
AltSaved = Xsaved(:,3);

t = 0:dt:Nsamples*dt-dt;

figure
plot(t,PosSaved)

figure
plot(t,VelSaved)

figure
plot(t,AltSaved)

EstimatedDistance = sqrt(PosSaved.^2 + AltSaved.^2);

% 레이더로 측정된 거리와 EKF로 추정된 거리 비교 플롯
figure
plot(t, Zsaved, 'r', 'DisplayName', 'Radar Measured')
hold on
plot(t, EstimatedDistance, 'b', 'DisplayName', 'EKF Estimated')
xlabel('Time [sec]')
ylabel('Radar Distance [m]')
legend