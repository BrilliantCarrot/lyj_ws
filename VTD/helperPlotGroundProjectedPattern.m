function helperPlotGroundProjectedPattern( clut )
% Input a ClutterGenerator that has an associated radarTransceiver. Plots
% the clutter regions and ground-projected gain pattern. Assumes a flat
% infinite ground plane at Z=0.

Naz = 360*4;
Nel = 90*4;

% Force update the patch generator sensor data
clut.PatchGenerator.updateSensorData(clut.Platform,clut.Radar,clut.Scenario.SimulationTime,clut.UseBeam);

pos = clut.PatchGenerator.SensorData.Position;
fc = clut.PatchGenerator.SensorData.CenterFrequency;
B = clut.PatchGenerator.SensorData.SensorFrame;
maxGndRng = clut.RangeLimit;

% Get azimuth/elevation grid in scenario coordinates
azScen = linspace(-180,180,Naz);
maxEl = -atand(pos(3)/maxGndRng);
elScen = linspace(-90,maxEl,Nel);

% Convert az/el to sensor frame
[losxScen,losyScen,loszScen] = sph2cart(azScen*pi/180,elScen.'*pi/180,1);
R = B.';
losx = R(1,1)*losxScen + R(1,2)*losyScen + R(1,3)*loszScen;
losy = R(2,1)*losxScen + R(2,2)*losyScen + R(2,3)*loszScen;
losz = R(3,1)*losxScen + R(3,2)*losyScen + R(3,3)*loszScen;
[az,el,~] = cart2sph(losx,losy,losz);
az = az*180/pi;
el = el*180/pi;

% Get gain pattern
sensor = clut.Radar.TransmitAntenna.Sensor;
G = sensor(fc,[az(:) el(:)].');
G = reshape(G,size(az));
G = G/max(G(:));

rtg = -pos(3)./loszScen;
surf(losxScen.*rtg,losyScen.*rtg,zeros(size(az)),20*log10(abs(G)))
axis equal
shading flat
view(0,90)
clim([-60 0])

hold on
for reg = clut.Regions
    helperPlotClutterRegion(reg,pos);
end
hold off

end