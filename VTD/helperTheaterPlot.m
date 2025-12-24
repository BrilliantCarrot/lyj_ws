function helperTheaterPlot(clut,params)

arguments
    clut
    params.Parent = []
    params.Detections = []
    params.ShowPatches = false
end

if isempty(params.Parent)
    % Make new figure for theater plot
    figure
    tp = theaterPlot('Parent',gca);
else
    % Use specified axes
    cla(params.Parent)
    tp = theaterPlot('Parent',params.Parent);
end

% Find targets
tgts = clut.Scenario.Platforms(2:end);
if ~isempty(tgts)
    tgtPos = cell2mat(cellfun(@(t) t.Position,tgts.','UniformOutput',0));
end

% Get detection positions
if ~isempty(params.Detections)
    detPos = cell2mat(cellfun(@(t) t.Measurement(1:3).',params.Detections,'UniformOutput',0));
end

% Make plotters
if ~isempty(tgts)
    platPlotter = platformPlotter(tp,'DisplayName','Target','Marker','+','MarkerEdgeColor','r');
end
if ~isempty(params.Detections)
    detPlotter = detectionPlotter(tp,'DisplayName','Radar Detections','Marker','.','MarkerEdgeColor','magenta','MarkerSize',4);
end
clutPlotter = clutterRegionPlotter(tp,'DisplayName','Clutter Region','ShowPatchCenters',params.ShowPatches);
surfPlotter = surfacePlotter(tp,'DisplayName','Scenario Surface');

% Do plotting
if ~isempty(tgts)
    plotPlatform(platPlotter,tgtPos);
end
plotHeight = 1;
plotClutterRegion(clutPlotter,clutterRegionData(clut,plotHeight))
if ~isempty(params.Detections)
    plotDetection(detPlotter,detPos)
end
plotSurface(surfPlotter,surfacePlotterData(clut.Scenario.SurfaceManager))

end