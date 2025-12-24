function helperPlotClutterScenario( scene,dets,useECEF,ax )
% This helper function assists with visualization of radar scenario clutter
% simulation

if nargin < 3
    useECEF = false;
end

% LLA origin for plotting in ENU
llaOrigin = [];
if ~useECEF
    if scene.IsEarthCentered
        if isempty(scene.SurfaceManager.Surfaces)
            if isempty(scene.Platforms)
                llaOrigin = [0;0;0];
            else
                llaOrigin = [scene.Platforms{1}.Position(1:2).'; 0];
            end
        else
            % If there's a bounded surface, use the first one's origin
            for ind = 1:numel(scene.SurfaceManager.Surfaces)
                if scene.SurfaceManager.Surfaces(ind).isBounded
                    llaOrigin = scene.SurfaceManager.Surfaces(ind).LocalOrigin;
                    break
                end
            end
            % If no surface origins, use a platform or default to all-zeros
            if isempty(llaOrigin)
                if isempty(scene.Platforms)
                    llaOrigin = [0; 0; 0];
                else
                    llaOrigin = scene.Platforms{1}.Position.';
                    llaOrigin(3) = 0; % place on ground
                end
            end
        end
    end
end

if nargin < 4
    % Create a new figure
    figure
else
    axes(ax)
end

% Plot surfaces
for ind = 1:numel(scene.SurfaceManager.Surfaces)
    if scene.IsEarthCentered
        helperPlotScenarioSurface( scene.SurfaceManager.Surfaces(ind),llaOrigin,useECEF )
    else
        helperPlotScenarioSurface( scene.SurfaceManager.Surfaces(ind) )
    end
    hold on
end

% Plot platforms
plats = scene.Platforms;
for ind = 1:numel(plats)
    if isempty(plats{ind}.Sensors)
        % Plot target platform
        pos = plats{ind}.Position;
        if scene.IsEarthCentered
            if useECEF
                pos = fusion.internal.frames.lla2ecef(pos);
            else
                pos = fusion.internal.frames.lla2enu(pos,llaOrigin.');
            end
        end
        plot3(pos(1),pos(2),pos(3),'+r');
        hold on
        continue
    end
    
    % 클러터의 정보가 clutter에 담겨있음
    % 그러므로 clutter에 대해서 이 함수에서 시각화를 수행
    clutter = scene.getClutterGenerator(plats{ind}.Sensors{1});
    
    if isempty(clutter)
        continue
    end

    % Quiver scale based on 3rd coordinate of position
    s = clutter.PatchGenerator.SensorData.Position(3)/2;
    
    % Get flat sensor data
    if scene.IsEarthCentered
        sd0 = clutter.PatchGenerator.SensorData;
        sdata = radar.internal.scenario.PatchGeneratorSensorData;

        % These properties are independent of Earth model
        sdata.CenterFrequency = sd0.CenterFrequency;
        sdata.CenterWavelength = sd0.CenterWavelength;
        sdata.Beam = sd0.Beam;
        sdata.LookAngleSensor = sd0.LookAngleSensor;

        % Convert position, velocity, and t2he sensor frame basis to ECEF
        sdata.Position = fusion.internal.frames.lla2ecef(sd0.Position.');
        R = fusion.internal.frames.enu2ecefrotmat(sd0.Position(1),sd0.Position(2));
        sdata.Velocity = R*sd0.Velocity;
        sdata.SensorFrame = R*sd0.SensorFrame;

        if ~useECEF
            % If not using ECEF, convert to the local ENU
            sdata.Position = fusion.internal.frames.ecef2enu(sdata.Position.',llaOrigin.');
            R = fusion.internal.frames.ecef2enurotmat(llaOrigin(1),llaOrigin(2));
            sdata.Velocity = R*sdata.Velocity;
            sdata.SensorFrame = R*sdata.SensorFrame;
        end
    else
        sdata = clutter.PatchGenerator.SensorData;
    end
    
    % Plot last patch data
    if clutter.LastPatchData.hasPatches
        lastPatches = clutter.LastPatchData.Centers;
        if scene.IsEarthCentered
            if useECEF
                lastPatches = fusion.internal.frames.lla2ecef(lastPatches.').';
            else
                lastPatches = fusion.internal.frames.lla2enu(lastPatches.',llaOrigin.').';
            end
        end
        plot3(lastPatches(1,:),lastPatches(2,:),lastPatches(3,:),...
            'Marker','.','LineStyle','none','MarkerEdgeColor','green','markersize',6);
    end
    
    pos = sdata.Position;
    vel = sdata.Velocity;
    
    % Position and velocity
    plot3(pos(1),pos(2),pos(3),'oblack','MarkerSize',7)
    hold on
    quiver3(pos(1),pos(2),pos(3),vel(1),vel(2),vel(3),'magenta','autoscale','off')
    text(pos(1),pos(2),pos(3)+60,sprintf('Platform %d',plats{ind}.PlatformID))
    
    % Boresight vector
    bs = sdata.Boresight;
    quiver3(pos(1),pos(2),pos(3),bs(1)*s,bs(2)*s,bs(3)*s,'cyan','autoscale','off');
    
    % Radar frame basis vectors
    B = sdata.SensorFrame;
    quiver3(pos(1),pos(2),pos(3),B(1,1)*s,B(2,1)*s,B(3,1)*s,'r','autoscale','off','linewidth',2);
    quiver3(pos(1),pos(2),pos(3),B(1,2)*s,B(2,2)*s,B(3,2)*s,'g','autoscale','off','linewidth',2);
    quiver3(pos(1),pos(2),pos(3),B(1,3)*s,B(2,3)*s,B(3,3)*s,'b','autoscale','off','linewidth',2);
    
end

% Plot detection positions (requires DetectionCoordinates be 'scenario')
if nargin > 1 && ~isempty(dets)
    if iscell(dets)
        detpos = cell2mat(cellfun(@(t) t.Measurement(1:3),dets.','UniformOutput',0));
    else
        detpos = [dets.Measurement];
    end
    if scene.IsEarthCentered && ~useECEF
        detpos = fusion.internal.frames.ecef2enu(detpos.',llaOrigin.').';
    end    
    plot3(detpos(1,:),detpos(2,:),detpos(3,:),'.magenta');
end

% Apply axis labels
if scene.IsEarthCentered
    if useECEF
        xlabel('X (ECEF)')
        ylabel('Y (ECEF)')
        zlabel('Z (ECEF)')
    else
        xlabel('East')
        ylabel('North')
        zlabel('Up')
    end    
else
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
end

hold off
grid on
axis equal

end