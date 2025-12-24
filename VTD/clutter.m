clear;
clc;
close all;

%% 1. Configure Scenario for Clutter Generation
% Introduction to Radar Scenario Clutter Simulation

% This example shows how to generate monostatic surface clutter signals and detections in a radar scenario. 
% Clutter detections will be generated with a monostatic radarDataGenerator, 
% and clutter return signals will be generated 
% with a radarTransceiver, using both homogenous surfaces and real terrain data from a DTED file. 
% theaterPlot is used to visualize the scenario surface and clutter generation.

% Configure Scenario for Clutter Generation
% Configuration of a radar scenario to simulate surface clutter involves creating a radarScenario object, 
% adding platforms with mounted radars, adding surface objects 
% that define the physical properties of the scenario surface, 
% and enabling clutter generation for a specific radar in the scene.
% Select a Radar Model

% 레이더 스캔 범위 조정을 위해 바꾸어야할 파라미터 목록
% range bin의 생성 관련
% 1. prf: 펄스 반복 주파수
% 2. rngRes: 레이더의 range bins를 생성할 구분 거리
% 레이더의 실제 스캐닝 범위 관련
% 1. fov: 레이더의 스캔 폭
% 2. clutRngLimit: 클러터의 최대 해석 거리
% 부가 요소: angRes, clutRes, beamwidth3dB, beamwidthNN, 

mountAng = [-90 10 0];
fc = 5e9;
rngRes = 150;       % doppler bins이 생성될 거리 기준
prf = 12e3;
% prf = 2.3e3;

numPulses = 64;

% The radarDataGenerator is a statistical model that does not directly emulate an antenna pattern. Instead, 
% it has properties that define the field of view and angular resolution. Use 10 degrees for the field of view and 
% angular resolution in each direction. This configuration is 
% comparable to simulating a single mainlobe with no angle estimation.

fov = [10 10];      % fov를 통해 레이더가 보는 폭을 조절
angRes = fov;

c = physconst('lightspeed');
lambda = freq2wavelen(fc);
rangeRateRes = lambda/2*prf/numPulses;
unambRange = time2range(1/prf);
unambRadialSpd = dop2speed(prf/4,lambda);
cpiTime = numPulses/prf;
rdr = radarDataGenerator(1,'No scanning','UpdateRate',1/cpiTime,'MountingAngles',mountAng,...
    'DetectionCoordinates','Scenario','HasINS',true,'HasElevation',true,'HasFalseAlarms',true, ...
    'HasRangeRate',true,'HasRangeAmbiguities',true,'HasRangeRateAmbiguities',true, ...
    'MaxUnambiguousRadialSpeed',unambRadialSpd,'MaxUnambiguousRange',unambRange,'CenterFrequency',fc, ...
    'FieldOfView',fov,'AzimuthResolution',angRes(1),'ElevationResolution',angRes(2), ...
    'RangeResolution',rngRes,'RangeRateResolution',rangeRateRes);

% Create a Scenario

scenario = radarScenario('UpdateRate',0,'IsEarthCentered',false);

% 레이더에 대해 초기화

rdrAlt = 1.5e3;
rdrSpd = 70;
rdrDiveAng = 10;
rdrPos = [0 0 rdrAlt];
rdrVel = rdrSpd*[0 cosd(rdrDiveAng) -sind(rdrDiveAng)];
rdrOrient = rotz(90).';
rdrTraj = kinematicTrajectory('Position',rdrPos,'Velocity',rdrVel,'Orientation',rdrOrient);
rdrplat = platform(scenario,'Sensors',rdr,'Trajectory',rdrTraj);

% Define the Scenario Surface

% Create a simple unbounded land surface with a constant-gamma reflectivity model. 
% Use the surfaceReflectivityLand function 
% to create a reflectivity model and attach the reflectivity model 
% to the surface with the RadarReflectivity parameter. 
% Use a gamma value of -20 dB.

% gamma값에 대해 일정한 값 사용

refl = surfaceReflectivityLand('Model','ConstantGamma','Gamma',-10);    % Wooded Hill의 gamma값 -10 적용
srf = landSurface(scenario,'RadarReflectivity',refl);

% The ReferenceHeight property gives the constant height of the surface when no terrain is specified, 
% or the origin height to which terrain is referenced if terrain is specified. 
% The ReflectivityMap property is relevant only 
% when a custom reflectivity model is used, and allows different reflectivity curves to be associated to 
% different parts of the surface. The Boundary property 
% gives the rectangular boundary of the surface in two-point form. 
% Elements of Boundary can be +/-inf to indicate the surface is unbounded in one or more directions. 
% Check the boundary of the surface created above to see that it is unbounded in all directions.
srf.Boundary

% Access the SurfaceManager property of the scenario to see the surface objects that have been added, 
% as well as any additional options related to the scenario surface.
scenario.SurfaceManager

% Enable Clutter Generation
% Clutter Generator

clutRes = rngRes/2;
% clutRngLimit = 24e3;
clutRngLimit = 12e3;
clut = clutterGenerator(scenario,rdr,'Resolution',clutRes,'UseBeam',true,'RangeLimit',clutRngLimit);

% The radarDataGenerator is a statistics-based detectability simulator, and only simulates 
% mainlobe detections within the field of view. As such, having UseBeam of the ClutterGenerator set to true is 
% sufficient to completely capture the effect of clutter interference on the detectability of target platforms 
% when using a radarDataGenerator.

%% 2. Visualize and Run Scenario
% Theater Plotter
% The theaterPlot object can be used along with a variety of theater plotters 
% to create customizable visual representations 
% of the scenario. Start by creating the theater plot.

tp = theaterPlot;

% Now create plotter objects for the scenario surface, clutter regions, and resulting radar detections. 
% The values specified for the DisplayName properties are used for the legend entries.

surfPlotter = surfacePlotter(tp,'DisplayName','Scenario Surface');
clutPlotter = clutterRegionPlotter(tp,'DisplayName','Clutter Region');
detPlotter = detectionPlotter(tp,'DisplayName','Radar Detections','Marker','.', ...
    'MarkerEdgeColor','magenta','MarkerSize',4);

dets = detect(scenario);

% Plot the clutter region, which in this case is simply the beam footprint, along with the detection positions. 
% Since the land surface used here is unbounded, 
% the plotSurface call should come last so that the surface plot extends 
% over the appropriate axis limits. The clutterRegionData method on the clutter generator is used to get plot data 
% for the clutter region plotter. Similarly, for the surface plotter, 
% the surfacePlotterData method on the scenario surface manager is used.

% plotClutterRegion(clutPlotter,clutterRegionData(clut))
% detpos = cell2mat(cellfun(@(t) t.Measurement(1:3).',dets,'UniformOutput',0));
% plotDetection(detPlotter,detpos)
% plotSurface(surfPlotter,surfacePlotterData(scenario.SurfaceManager))

%% 3. Simulate Clutter IQ Signals
% Now you will create a radarTransceiver with similar radar system parameters and 
% simulate clutter at the signal level. 
% The function helperMakeTransceiver is provided to quickly create a transceiver 
% with the desired system parameters.

% Define the desired beamwidth. For comparison to the above scenario, simply 
% let the beamwidth equal the field of view that was used.

beamwidth3dB = fov;

useCustomElem = true;
rdriq = helperMakeTransceiver(beamwidth3dB,fc,rngRes,prf,useCustomElem);

rdriq.MountingAngles = mountAng;
rdriq.NumRepetitions = numPulses;

% Re-create the same scenario, using this new radar model. 
% Start by calling release on System Objects that will be re-used.

release(rdrTraj)
scenario = radarScenario('UpdateRate',0,'IsEarthCentered',false);
platform(scenario,'Sensors',rdriq,'Trajectory',rdrTraj);
landSurface(scenario,'RadarReflectivity',refl);

% Enable clutter generation for the radar. This time, disable the beam footprint clutter region 
% in favor of a custom ring-shaped region.

clutterGenerator(scenario,rdriq,'Resolution',clutRes,'UseBeam',false,'RangeLimit',clutRngLimit);

% If the clutterGenerator method was called without any output argument, as above, 
% the handle to the constructed ClutterGenerator may still be found with the scenario getClutterGenerator method 
% by passing in a handle to the associated radar.

% clut = getClutterGenerator(scenario,rdriq);

% After creating the ClutterGenerator, you can use the ringClutterRegion method 
% to create a null-to-null footprint region 
% for clutter generation. Use a simple estimate of the 
% null-to-null beamwidth as about 2.5 times the 3 dB beamwidth, 
% then find the minimum elevation angle to encompass the near edge of the beam, and 
% finally convert that to a minimum ground range for the region.
beamwidthNN = 2.5*beamwidth3dB;
minel = -mountAng(2) - beamwidthNN(2)/2;
minrad = -rdrAlt/tand(minel);
% For the max radius parameter, simply find the ground range corresponding 
% to the clutter range limit specified earlier.
maxrad = sqrt(clut.RangeLimit^2 - rdrAlt^2);
% The azimuth span will equal the null-to-null beamwidth, and the azimuth center will be 0 degrees 
% since the beam is pointing along the +X direction in scenario coordinates.
azspan = beamwidthNN(1);
azc = 0;
ringClutterRegion(clut,minrad,maxrad,azspan,azc)

% Using the provided helper function, plot the ground-projected antenna pattern along with the 
% ring clutter region you just created. The ring region created above nicely encompasses the entire mainlobe.

% helperPlotGroundProjectedPattern(clut)

% Run the simulation again for one frame, this time using the scenario receive method to simulate IQ signals.

iqsig = receive(scenario);
PH = iqsig{1};

% Since the radarTransceiver used a single custom element, the resulting signal will be formatted with 
% fast-time samples along the first dimension and pulse index (slow time) along the second dimension. 
% This is the phase history (PH) matrix. Plot a DC-centered range-Doppler map (RDM) 
% using the helperPlotRDM function.

% figure
% helperPlotRDM(PH,rngRes,prf,numPulses)
% 
% helperTheaterPlot(clut)

%% 4. Simulate Surface Range Profiles for a Scanning Radar
% The automatic mainlobe clutter option supports scanning radars. In this section you will re-create the scenario 
% to use a stationary scanning linear array that collects a single pulse per scan position. 
% You will add a few stationary surface targets and view the resulting range profiles.
% Start by re-creating the radar object. This time, only pass the azimuth beamwidth to the helper function, 
% which indicates a linear array should be used. The custom element cannot be used for a linear array 
% if the automatic mainlobe clutter option is being used, 
% so that the ClutterGenerator has knowledge of the array geometry. 
% Reduce the range resolution to 75 meters to reduce the clutter power in gate.

useCustomElem = false;
rngRes = 75;
rdriq = helperMakeTransceiver(beamwidth3dB(1),fc,rngRes,prf,useCustomElem);

numPulses = 1;
rdriq.MountingAngles = mountAng;
rdriq.NumRepetitions = numPulses;

rdriq.ElectronicScanMode = 'Sector';
rdriq.ElectronicScanLimits = [-30 30;0 0];      % 범위 변경 가능
rdriq.ElectronicScanRate = [prf; 0];

scenario = radarScenario('UpdateRate',0,'IsEarthCentered',false,'StopTime',60/prf);     % prf앞의 숫자 단위를 변경
platform(scenario,'Sensors',rdriq,'Position',rdrPos,'Orientation',rotz(90).');
landSurface(scenario,'RadarReflectivity',refl);

clutterGenerator(scenario,rdriq,'Resolution',clutRes,'UseBeam',true,'RangeLimit',clutRngLimit);

tgtRCS = 40; % dBsm
platform(scenario,'Position',[8e3 -2e3 0],'Signatures',rcsSignature('Pattern',tgtRCS));
platform(scenario,'Position',[8e3    0 0],'Signatures',rcsSignature('Pattern',tgtRCS));
platform(scenario,'Position',[8e3  2e3 0],'Signatures',rcsSignature('Pattern',tgtRCS));

rangeGates = (0:ceil((unambRange-rngRes)/rngRes))*rngRes;
frame = 0;


%% 5. Simulate Smooth Surface Clutter for a Range-Doppler Radar
% Up till now you have simulated surface clutter using the "uniform" scatterer distribution mode. 
% For flat-Earth scenarios,
% the radarTransceiver radar model, and smooth surfaces (no terrain or spectral model associated with the surface),
% a faster range-Doppler-adaptive mode is available which uses a minimal number of clutter scatterers
% and a more accurate calculation of the clutter power in each range-Doppler resolution cell.

% Re-create the radarTransceiver, again with a linear array. 
% The automatic mainlobe region will not be used in this section,
% so use a custom element to speed things up.

useCustomElem = true;
rdriq = helperMakeTransceiver(beamwidth3dB(1),fc,rngRes,prf,useCustomElem);

numPulses = 64;
rdriq.MountingAngles = mountAng;
rdriq.NumRepetitions = numPulses;

scenario = radarScenario('UpdateRate',0,'IsEarthCentered',false);
landSurface(scenario,'RadarReflectivity',refl);

release(rdrTraj)
platform(scenario,'Sensors',rdriq,'Trajectory',rdrTraj);

clut = clutterGenerator(scenario,rdriq,'ScattererDistribution','RangeDopplerCells', ...
    'UseBeam',false,'RangeLimit',clutRngLimit);
ringClutterRegion(clut,minrad,maxrad,60,0);

platform(scenario,'Position',[8e3 -2e3 0],'Signatures',rcsSignature('Pattern',tgtRCS));
platform(scenario,'Position',[8e3    0 0],'Signatures',rcsSignature('Pattern',tgtRCS));
platform(scenario,'Position',[8e3  2e3 0],'Signatures',rcsSignature('Pattern',tgtRCS));

iqsig = receive(scenario);
PH = iqsig{1};
% helperPlotRDM(PH,rngRes,prf,numPulses);

%% 6. Clutter from Terrain Data 
% 지형 클러터 생성 단계로 1,3만 필요
% -------------------------------------------------------------------------------------------------------------- %
% In the previous sections, you simulated homogeneous clutter from an unbounded flat surface. In this section, 
% you will use a DTED file to simulate clutter return from real terrain data in an Earth-centered scenario. 
% You will collect two frames of clutter return - one with shadowing enabled and 
% one without shadowing, and compare the results.
% Start by creating the scenario, this time setting the IsEarthCentered flag to true in order to use a DTED file, 
% which consists of surface height samples over a latitude/longitude grid.


close all;

scenario = radarScenario('UpdateRate',0,'IsEarthCentered',true);

% refLLA_test = [39.43; -105.84];
% bdry_test = refLLA_test + [0 1;-1/2 1/2]*0.15;

refLLA = [38.5001; 127.4999];
bdry = refLLA + [0 1;-1/2 1/2]*0.3;        % refLLA 좌표를 토대로 주변 영역을 포함하는 바운더리 설정

% srf = landSurface(scenario,'Terrain','n39_w106_3arc_v2.dt1','Boundary',bdry_test,'RadarReflectivity',refl);
srf = landSurface(scenario,'Terrain','NK_DTED.dt2','Boundary', bdry, 'RadarReflectivity', refl);

rdrAlt = 1500;
% rdrAlt = 3000;      % 고도를 높이면 더 잘보임

% srfHeight = height(srf,refLLA);
% rdrAlt = srfHeight + rdrAlt;
rdrPos1 = [refLLA; rdrAlt];     % 레이더의 위치

rdrVelWest = [-rdrSpd 0 0];

toa = [0;1];    % Times of arrival at each waypoint
% 레이더 속도가 ENU로 표현되었기에 LLA로 변환(DEM 형식)
% 시간이 지난 후의 레이더 위치
rdrPos2 = enu2lla(rdrVelWest,rdrPos1.','ellipsoid').';  

rdrTrajGeo = geoTrajectory('Waypoints',[rdrPos1, rdrPos2].','TimeOfArrival',toa,'ReferenceFrame','ENU');
platform(scenario,'Sensors',rdriq,'Trajectory',rdrTrajGeo);

clut = clutterGenerator(scenario,rdriq,'Resolution',clutRes,'UseBeam',false,'RangeLimit',clutRngLimit);

ringClutterRegion(clut,minrad,maxrad,azspan,azc);

iqsig_withShadow = receive(scenario);
PH_withShadowing = iqsig_withShadow{1};
helperPlotClutterScenario(scenario)
title('Clutter patches - with terrain shadowing')

clut.UseShadowing = false;
iqsig_noShadow = receive(scenario);
PH_noShadowing = iqsig_noShadow{1};
helperPlotClutterScenario(scenario)
title('Clutter patches - without terrain shadowing')

% Range-Doppler Map 상에서 [dbw]로 표현되는 신호의 세기를 시각화
figure
% subplot(1,2,1)
helperPlotRDM(PH_withShadowing,rngRes,prf,numPulses)
title('RDM - with terrain shadowing')
figure
% subplot(1,2,2)
helperPlotRDM(PH_noShadowing,rngRes,prf,numPulses)
title('RDM - without terrain shadowing')
% set(gcf,'Position',get(gcf,'Position')+[0 0 560 0])

% Conclusion
% In this example, you saw how to configure a radar scenario to include 
% clutter return as part of the detect and receive methods,
% generating clutter detections and IQ signals with the radarDataGenerator and radarTransceiver, respectively. 
% You saw how to define a region of the scenario surface with an associated reflectivity model, 
% and how to specify regions of interest for clutter generation. Surface shadowing is simulated 
% when generating clutter returns from surfaces with terrain, and a faster range-Doppler-adaptive mode can be used 
% for flat-Earth scenarios with smooth surfaces.

%% << CFAR Detection >>
%% 1. 지형 데이터 Import
% (80km*80km의 경우 데이터 로드 5분 소요)

close all;
% csv 파일로부터 각 셀이 탭으로 구분된 데이터를 생성
data = readtable(['C:/Users/leeyj/OneDrive - 인하대학교/School/과제/[국방 수직이착륙기 특화연구센터(VTD-13)]/' ...
    '자료/NK_Flippped_DEM_2.csv'], 'Delimiter', '\t', 'ReadVariableNames', false);
% 2. 경도, 위도, 고도 데이터를 담을 빈 행렬 선언(원본 데이터와 동일 크기)
longitude_table = zeros(height(data), width(data));
latitude_table = zeros(height(data), width(data));
altitude_table = zeros(height(data), width(data));
% 3. 각 행을 반복하며 경도, 위도, 고도 데이터를 추출
for i = 1:height(data)      % 30 행
    % 한 행의 데이터를 가져오기
    row_data = table2array(data(i,:));       % 한 행으로만 이루어진 row_data 변수
    % 각 열에 대해 반복
    for j = 1:width(row_data)       % 20 열
        % 각 셀 데이터를 문자형으로 변환
        current_data = char(row_data{j});
        % 데이터를 쉼표로 분리하고 문자를 수치로 변경하여 경도, 위도, 고도 정보 담긴 테이블로 저장
        splitted_data = strsplit(current_data, ',');
        splitted_data = str2double(splitted_data);
        longitude_table(i,j) = splitted_data(1);
        latitude_table(i,j) = splitted_data(2);
        altitude_table(i,j) = splitted_data(3);
    end
end

%% 2. 클러터 생성에 필요한 파라미터 입력 및 실행

% 범위를 제한하여 바운더리 설정 후 해당 영역에서 클러터 시각화
refLLA = [mean(latitude_table(:)); mean(longitude_table(:))];
bdry = refLLA + [0 1;-1/2 1/2]*0.15;

mountAng = [0 0 0];
fc = 5e9;
rngRes = 150;
prf = 12e3;
numPulses = 64;
fov = [10 10];
beamwidth3dB = fov;
angRes = fov;
c = physconst('lightspeed');
lambda = freq2wavelen(fc);
rangeRateRes = lambda/2*prf/numPulses;
unambRange = time2range(1/prf);
unambRadialSpd = dop2speed(prf/4,lambda);
cpiTime = numPulses/prf;
rdr = radarDataGenerator(1,'No scanning','UpdateRate',1/cpiTime,'MountingAngles',mountAng,...
    'DetectionCoordinates','Scenario','HasINS',true,'HasElevation',true,'HasFalseAlarms',false, ...
    'HasRangeRate',true,'HasRangeAmbiguities',true,'HasRangeRateAmbiguities',true, ...
    'MaxUnambiguousRadialSpeed',unambRadialSpd,'MaxUnambiguousRange',unambRange,'CenterFrequency',fc, ...
    'FieldOfView',fov,'AzimuthResolution',angRes(1),'ElevationResolution',angRes(2), ...
    'RangeResolution',rngRes,'RangeRateResolution',rangeRateRes);

scenario = radarScenario('UpdateRate',0,'IsEarthCentered',true);


rdrLon = 127.4999;      % 평균 경도
rdrLat = 38.5001;        % 평균 위도
rdrAlt = 471.7196;    % 평균 고도
% rdrAlt = 1500.7196;    % 평균 고도
rdrSpd = 0;         % 고정형 레이더
rdrDiveAng = 0;
rdrPos = [rdrLat; rdrLon; rdrAlt];
rdrVel = [0 0 0];       % 고정형 레이더
rdrOrient = rotz(90).';
% rdrTraj = kinematicTrajectory('Position',rdrPos,'Velocity',rdrVel,'Orientation',rdrOrient);
% platform(scenario,'Sensors',rdr,'Trajectory',rdrTraj);

refl = surfaceReflectivityLand('Model','ConstantGamma','Gamma',-20);
srf = landSurface(scenario,'Terrain','NK_DTED.dt2','Boundary', bdry, 'RadarReflectivity', refl);
clutRes = rngRes/2;
clutRngLimit = 12e3;

useCustomElem = true;
rdriq = helperMakeTransceiver(beamwidth3dB,fc,rngRes,prf,useCustomElem);
rdriq.MountingAngles = mountAng;
rdriq.NumRepetitions = numPulses;

beamwidthNN = 2.5*beamwidth3dB;
minel = -mountAng(2) - beamwidthNN(2)/2;
minrad = -rdrAlt/tand(minel);

maxrad = sqrt(clut.RangeLimit^2 - rdrAlt^2);

azspan = beamwidthNN(1);
azc = 0;

rdrPos1 = rdrPos;
rdrVelWest = [-rdrSpd 0 0];
toa = [0; 1];
rdrPos2 = enu2lla(rdrVelWest, rdrPos1.', 'ellipsoid').';
rdrTrajGeo = geoTrajectory('Waypoints', [rdrPos1, rdrPos2].', 'TimeOfArrival', toa, 'ReferenceFrame', 'ENU');

platform(scenario, 'Sensors', rdriq, 'Trajectory', rdrTrajGeo);

clut = clutterGenerator(scenario,rdr,'Resolution',clutRes,'UseBeam',true,'RangeLimit',clutRngLimit);

%% 3. DTED에 대해 시각화

ringClutterRegion(clut,minrad,maxrad,azspan,azc);

helperPlotClutterScenario(scenario)
title('Clutter patches - with terrain shadowing')

iqsig = receive(scenario);
PH_withShadowing = iqsig{1};
figure
subplot(1,2,1)
helperPlotRDM(PH_withShadowing,rngRes,prf,numPulses)
title('RDM - with terrain shadowing')


% 바운더리 설정용 임시 코드
% refLLA = [mean(latitude_table(:)); mean(longitude_table(:))];
% bdry = refLLA + [0 1;-1/2 1/2]*0.15;


%% << Radar Performance Analysis over Terrain >>


rdrppower = 1e3;            % Peak power (W)
fc = 6e9;                   % Operating frequency (Hz)
hpbw = [2; 5];              % Half-power beamwidth [azimuth; elevation] (deg)
rdrpulsew = 1e-6;           % Pulse width (s)
lambda = freq2wavelen(fc);  % Wavelength (m)


rdrgain = beamwidth2gain(hpbw,"CosineRectangular"); % Transmitter and receiver gain (dB)


rdrlat = 39.913756;         % Radar latitude (deg)
rdrlon = -105.118062;       % Radar longitude (deg)
rdrtowerht = 10;            % Antenna height (m)
rdralt = 1717 + rdrtowerht; % Radar altitude (m)


dtedfile = "n39_w106_3arc_v2.dt1";
attribution = "SRTM 3 arc-second resolution. Data available from the U.S. Geological Survey.";
[Zterrain,Rterrain] = readgeoraster(dtedfile,"OutputType","double");


% Visualize the location using the geographic globe plot.
% addCustomTerrain("southboulder",dtedfile,Attribution=attribution);
fig = uifigure;
g = geoglobe(fig,Terrain="southboulder");
hold(g,"on")
h_rdrtraj = geoplot3(g,rdrlat,rdrlon,rdralt,"ro",LineWidth=6,MarkerSize=10);


%%

tlat0 = 39.80384;           % Target initial latitude (deg)
tlon0 = -105.49916;         % Target initial longitude (deg)
tht0 = 3000;                % Target initial height (m)
azs = 1:2:540;              % Target azimuth (deg)
r = 5000;                   % Target slant range (m)

% Convert from polar coordinates to Cartesian East, North, Up (ENU).
[X,Y] = pol2cart(deg2rad(azs),r);

% Convert ENU to geodetic.
Z = linspace(0,1000,numel(azs));
wgs84 = wgs84Ellipsoid;
[tlat,tlon,tht] = enu2geodetic(X,Y,Z,tlat0,tlon0,tht0,wgs84);

% Define the target altitude relative to the geoid. 
talt = tht - egm96geoid(tlat,tlon); % Target altitude (m)


fs = 0.1;
t = (0:length(X)-1)/fs;
ttraj = geoTrajectory([tlat.' tlon.' tht.'],t,SampleRate=fs);


h_ttraj = geoplot3(g,tlat,tlon,talt,"yo",LineWidth=3);
campos(g,39.77114,-105.62662,6670)
camheading(g,70)
campitch(g,-12)


trcs = pow2db(10);          % Target RCS (dBsm)


scene = radarScenario(IsEarthCentered=true,UpdateRate=fs,StopTime=t(end));
rdrplatform = platform(scene,Position=[rdrlat,rdrlon,rdralt],Sensor=radarDataGenerator);
tplatform = platform(scene,Trajectory=ttraj,Signatures= ...
    {rcsSignature(Azimuth=[-180 180],Elevation=[-90 90],Pattern=trcs)});



temp = 21.1;                % Ambient temperature (degrees Celsius)
gwc = 0.3;                  % Gravimetric water content
[~,~,epsc] = earthSurfacePermittivity("vegetation",fc,temp,gwc);



tsnr = -inf(size(t));
F = zeros(size(t));
trange = zeros(size(t));
isVisible = false(size(t));
idx = 1;

while advance(scene)
    tpose = pose(tplatform,CoordinateSystem="Geodetic");
    tpos = tpose.Position;
    [isVisible(idx),~,~,h] = los2(Zterrain,Rterrain,rdrlat,rdrlon, ...
        tpos(1),tpos(2),rdralt,tpos(3),"MSL","MSL");
    hgtStdDev = std(h); 
    if isVisible(idx)
        trange(idx) = llarangeangle([rdrlat rdrlon rdralt],tpos,1); 
        F(idx) = radarpropfactor(trange(idx),fc,rdralt,tpos(3), ...
            SurfaceRelativePermittivity=epsc,...
            SurfaceHeightStandardDeviation=hgtStdDev, ...
            ElevationBeamwidth=hpbw(2));
    end
    idx = idx+1;
end


tsnr(isVisible) = radareqsnr(lambda,trange(isVisible).',rdrppower,rdrpulsew,...
    RCS=trcs,Gain=rdrgain,PropagationFactor=F(isVisible).');



tsnr_finiteidx = ~isinf(tsnr);
tsnr_cidx = zeros(size(tsnr));
cmap = colormap(g);
numclvls = size(cmap,1);
tsnr_cidx(tsnr_finiteidx) = discretize(tsnr(tsnr_finiteidx),numclvls-1);
tsnr_cidx(~tsnr_finiteidx) = numclvls;

delete(h_ttraj);
hsnr = zeros(size(tsnr));
for m = 1:numel(tsnr)
    hsnr(m) = geoplot3(g,tlat(m),tlon(m),talt(m), ...
        Marker="o",LineWidth=2,MarkerSize=1);
    if tsnr_finiteidx(m)
        set(hsnr(m),"Color",cmap(tsnr_cidx(m),:));
    else
        set(hsnr(m),"Color","r");
    end
end


%% <<<<< 클러터가 고려된 SIR 산출 >>>>>

clc; clear;

% CBSC = -15;
% height = 3;
% targetHeight = 100;

% 레이더 파라미터
% Pt = 6e3;                   % Peak power (W), 12kW
lambda = freq2wavelen(9e9); % Wavelength (m), Scanter 4000
% lambda = freq2wavelen(5.6e9); % Wavelength (m), 레이더 주파수 입력
% Pt = 114.6e3;                   % Peak power (W)
Pt = 12e3;                  % Scanter 4000의 Peak Power
Gt = 34;                    % Transmit antenna gain (dB)
Gr = 34;                    % Receive antenna gain (dB)
rcs = 9;                    % 헬기 RCS(dB)
rcs = 10^(rcs/10);          % 헬기 RCS, dB값을 log값으로 변환 
tau = 8.0e-8;               % pulse width, scanter 4000의 경우 80ns

% 노이즈 항
R = linspace(2e3, 50e3, 100).';  % 거리 (0에서 50km)
k = 1.38e-23;               % 볼츠만 상수 (J/K)
Ts = 290;                   % 시스템 잡음 온도 (K)
B = 1e6;                    % Bandwidth (Hz)
F = 10^(6 / 10);                    % Noise Figure (m^2)
L = 10^(10 / 10);                   % Radar Loss (m^2)

sigma_target = 7.9432;      % 목표물 RCS (m^2), 9dB에서 
% sigma_target = 2.5119;      % 목표물 RCS (m^2), 논문의 경우

% 지형의 클러터
% 교재의 계산식을 이용
% 대기의 경우 0.001 m^2으로, 지표면의 경우 5 m^2으로 설정
c = 3e8;                      % 전파 속도 (m/s)
pulse_width = 8.0e-8;         % 펄스 폭 (s), 80 ns
sigma_0 = 10^(-20/10);        % 클러터 산란계수 (선형(log) 스케일), -20 dB(Flatland)
theta_A = deg2rad(1);       % 방위각 빔폭 (rad)
theta_E = deg2rad(2);       % 고각 빔폭 (rad)
SL_rms = 10^(-20/10);         % 사이드로브의 RMS 수준 (선형(log) 스케일), -20 dB

% 높이 및 거리
% R = 35e3;                  % 단일 거리용 변수
Rg = R*cos(theta_E);        % slant range의 지표면 투영
h_t = 10000;                  % 목표물 높이
h_r = 5;                      % 레이더 높이 (m)
R_s = sqrt(Rg.^2 + (h_t - h_r)^2);
R_e = 6.371e6;                % 지구 반지름 (m)

% Radar Range Resolution(레이더 거리 해상도)
delta_Rg = c * pulse_width / 2;
% 안테나 이득 패턴(가우시안 안테나 패턴 가정하였을 시)
% G_theta = exp(-(2.776 * (theta_E/theta_E)^2));
G_theta = exp(-2.776 * ((theta_E + theta_A) / theta_E)^2);  % 고각 및 방위각 두개에 대한 이득
% 메인빔 클러터 면적 및 RCS 계산
A_MBc = delta_Rg * Rg * theta_A;
sigma_MBc = sigma_0 * A_MBc * G_theta^2;
% 사이드로브 클러터 면적 및 RCS 계산
A_SLc = delta_Rg * pi * Rg;
sigma_SLc = sigma_0 * A_SLc * SL_rms^2;
% 레이다 탐지 범위의 성분 중 지평선 축의 거리
R_h = sqrt((8 * R_e * h_r)/3);
% 총 클러터 RCS 계산
sigma_TOTc = (sigma_MBc + sigma_SLc) ./ (1 + (R_s / R_h).^4);
sigma_clutter = sigma_TOTc;        % 지형 클러터의 RCS (m^2), 논문의 경우 -20 dB

% dB로부터 선형 스케일로 변환(dB에서 m^2단위로 변환)
Gt_lin = 10^(Gt / 10);
Gr_lin = 10^(Gr / 10);

% SNR, CNR, SCR 정리
SNR = (Pt * Gt_lin * Gr_lin * sigma_clutter * lambda^2) ./ ((4 * pi)^3 * R.^4 * k * Ts * B * F * L);
SNR_dB = 10 * log10(SNR);
CNR = (Pt * Gt_lin * Gr_lin * sigma_target * lambda^2) ./ ((4 * pi)^3 * R.^4 * k * Ts * B * F * L);
CNR_dB = 10 * log10(CNR);
SCR = (Pt * Gt_lin * Gr_lin * sigma_target * lambda^2) ./ (Pt * Gt_lin * Gr_lin * sigma_clutter * lambda^2);
% SCR = SNR./CNR;
SCR_dB = 10 * log10(SCR);
SIR = 1./((1./SNR)+(1./SCR));       % 클러터의 영향이 고려된 목표물의 SNR 값을 SIR(SCNR)로 정의
SIR_dB = 10 * log10(SIR);

% Pc = (Pt * Gt_lin * Gr_lin * sigma_clutter * lambda^2) ./ ((4 * pi)^3 * R.^4);    % 클러터 파워 (Pc)
% Pn = k * Ts * B;    % 노이즈 파워 (Pn)
% CNR = Pc / Pn;
% CNR_dB = 10 * log10(CNR);
% Ptgt = (Pt * Gt_lin * Gr_lin * sigma_target * lambda^2) ./ ((4 * pi)^3 * R.^4);   % 타겟의 파워 (Ptgt)
% SCR = Ptgt / Pc;
% SCR_dB = 10 * log10(SCR);

figure;
hold on;
plot(R / 1e3, SNR_dB, 'b-','LineWidth', 1.5);
plot(R / 1e3, CNR_dB, 'k-', 'LineWidth', 1.5);
plot(R / 1e3, SIR_dB, 'r-', 'LineWidth', 1.5);
xlabel('Rs (Slant Range) in Km');
ylabel('dB');
title('2km 상공 항공기의 경우 클러터의 영향이 고려된 SNR 수치');
legend("SNR", "CNR","SIR")
grid on;

% SCR 및 CNR에 따라 레이더에서의 탐지를 위한 추가 SNR을 계산
% required_SNR_cluttered_dB = 10 * log10(1 + CNR / SCR);
% required_SNR_without_CNR = 10 * log10(1 + SCR);
% fprintf('Clutter-to-Noise Ratio: %.2f dB\n', CNR_dB);
% fprintf('Signal-to-Clutter Ratio: %.2f dB\n', SCR_dB);
% fprintf('CNR과 SCR을 전부 고려하였을 시 요구되는 추가 SNR: %.2f dB\n', required_SNR_cluttered_dB);
% fprintf('CNR을 고려 안하였을 시 요구되는 추가 SNR: %.2f dB\n', required_SNR_without_CNR);

%% sigma c 값

c = 3e8;                      % 전파 속도 (m/s)
lambda = freq2wavelen(3e9);   % 파장 (m), 3 GHz
pulse_width = 8.0e-8;         % 펄스 폭 (s), 80 ns
sigma_0 = 10^(-20/10);        % 클러터 산란계수 (선형 스케일), -20 dB
% R = 35e3;                  % 거리
theta_A = deg2rad(1);       % 방위각 빔폭 (rad)
theta_E = deg2rad(2);       % 고각 빔폭 (rad)
SL_rms = 10^(-20/10);         % 사이드로브의 RMS 수준 (선형 스케일), -20 dB
Rg = R*cos(theta_E);        % slant range의 지표면 투영
h_r = 5;                      % 레이더 높이 (m)
R_e = 6.371e6;                % 지구 반지름 (m)
% Radar Range Resolution
delta_Rg = c * pulse_width / 2;
% 안테나 이득 패턴(가우시안 안테나 패턴 가정하였을 시)
% G_theta = exp(-(2.776 * (theta_E/theta_E)^2));
G_theta = exp(-2.776 * ((theta_E + theta_A) / theta_E)^2);  % 고각 및 방위각 두개에 대한 이득
% 메인빔 클러터 면적 및 RCS 계산
A_MBc = delta_Rg * Rg * theta_A;
sigma_MBc = sigma_0 * A_MBc * G_theta^2;
% 사이드로브 클러터 면적 및 RCS 계산
A_SLc = delta_Rg * pi * Rg;
sigma_SLc = sigma_0 * A_SLc * SL_rms^2;
% 레이다 탐지 범위의 성분 중 지평선 축의 거리
R_h = sqrt((8 * R_e * h_r)/3);
% 총 클러터 RCS 계산
sigma_TOTc = (sigma_MBc + sigma_SLc) / (1 + (R / R_h)^4);
% sigma_TOTc = (sigma_MBc + sigma_SLc) / (1 + (R / R_h)^4);

% 결과 출력
fprintf('메인빔 클러터 RCS (sigma_MBc): %.4f m^2\n', sigma_MBc);
fprintf('사이드로브 클러터 RCS (sigma_SLc): %.4f m^2\n', sigma_SLc);
fprintf('총 클러터 RCS (sigma_TOTc): %.4f m^2\n', sigma_TOTc);
sigma_TOTc_dB = 10*log10(sigma_TOTc);
fprintf('총 클러터 RCS의 dB 형태: %.4f dB',sigma_TOTc_dB);

%%

% 레이더 및 목표물 파라미터 설정
Pt = 12e3;                % 송신 전력 (W)
Gt_dB = 34;              % 안테나 이득 (dBi)
Gr_dB = 34;              % 안테나 이득 (dBi)
Gt = 10^(Gt_dB/10);      % 선형 값으로 변환
Gr = 10^(Gr_dB/10);
f = 8e9;              % 작동 주파수 (Hz)
c = 3e8;                 % 빛의 속도 (m/s)
lambda = freq2wavelen(f);          % 파장 (m)

sigma_target_dB = 9;     % 헬리콥터 RCS (dBsm)
sigma_target = 10^(sigma_target_dB/10);  % 선형 값 (m^2)

L_dB = 10;             % 시스템 손실 (dB)
L = 10^(L_dB/10);        % 선형 값

Fn_dB = 6;               % 잡음 지수 (dB)
Fn = 10^(Fn_dB/10);      % 선형 값

k = 1.38e-23;            % 볼츠만 상수 (J/K)
T = 290;                 % 시스템 온도 (K)
B = 25e6;                % 대역폭 (Hz)

% 클러터 파라미터 설정
sigma0_dB = -15;         % 클러터 반사율 (dB)
sigma0 = 10^(sigma0_dB/10);  % 선형 값

theta_az_deg = 1.8;      % 수평 빔 폭 (도)
theta_el_deg = 22;       % 수직 빔 폭 (도)
theta_az = deg2rad(theta_az_deg);  % 라디안으로 변환
theta_el = deg2rad(theta_el_deg);

% 거리 설정
R_km = [2, 5, 10, 20, 50];   % 거리 (km)
R = R_km * 1e3;              % 거리 (m)

% 결과 저장을 위한 배열 초기화
SNR_no_clutter_dB = zeros(size(R));
SNR_with_clutter_dB = zeros(size(R));

for i = 1:length(R)
    Ri = R(i);
    
    % 헬리콥터로부터의 수신 신호 전력 계산
    P_signal = (Pt * Gt * Gr * lambda^2 * sigma_target) / ((4 * pi)^3 * Ri^4 * L);
    
    % 클러터 면적 계산
    Ac = Ri^2 * theta_az * theta_el;
    
    % 클러터로부터의 수신 전력 계산
    P_clutter = (Pt * Gt * Gr * lambda^2 * sigma0 * Ac) / ((4 * pi)^3 * Ri^4 * L);
    
    % 열 잡음 전력 계산
    P_noise = k * T * B * Fn;
    
    % 클러터가 없는 경우의 SNR 계산
    SNR_no_clutter = P_signal / P_noise;
    
    % 클러터가 있는 경우의 SNR 계산
    SNR_with_clutter = P_signal / (P_noise + P_clutter);
    
    % 데시벨로 변환하여 저장
    SNR_no_clutter_dB(i) = 10 * log10(SNR_no_clutter);
    SNR_with_clutter_dB(i) = 10 * log10(SNR_with_clutter);
end

% 결과 시각화
figure;
plot(R_km, SNR_no_clutter_dB, 'bo-', 'LineWidth', 2);
hold on;
plot(R_km, SNR_with_clutter_dB, 'ro-', 'LineWidth', 2);
grid on;
xlabel('거리 (km)');
ylabel('SNR (dB)');
title('거리별 SNR 비교 (클러터 고려 여부)');
legend('클러터 미고려', '클러터 고려');

%% 결과 시각화

snr_values_1 = [9, 18.53, 27.53];
snr_values_2 = [9, 0, 9];
categories = {'기존 SNR', '목표물 탐지 SNR', '결과 SNR'};
figure;
b = bar([snr_values_1; snr_values_2]', 'grouped');
b(1).FaceColor = [0.65 0.16 0.16];  % 갈색
b(2).FaceColor = [0.53 0.81 0.98];  % 하늘색
% bar([snr_values_1; snr_values_2]', 'grouped');
set(gca, 'XTickLabel', categories);
ylabel('SNR 값 (dB)');
title('SNR 비교');
legend({'지표면 배경', '하늘 배경'}, 'Location', 'northwest');
grid on;

%% << Simulate Clutter for System with Known Power >> %%
% constantGammaClutter에 대한 매트랩 예시

% Simulate the clutter return from terrain with a gamma value of 0 dB. 
% The effective transmitted power of the radar system is 5 kW.

% Set up the characteristics of the radar system. This system uses a four-element uniform linear array (ULA). 
% The sample rate is 1 MHz, and the PRF is 10 kHz. The propagation speed is the speed of light, 
% and the operating frequency is 300 MHz. 
% The radar platform is flying 1 km above the ground with a path parallel to the ground along the array axis. 
% The platform speed is 2 km/s. The mainlobe has a depression angle of 30°.

clear;
clc;
close all;

Nele = 4;
c = physconst('Lightspeed');
fc = 300.0e6;
lambda = c/fc;
array = phased.ULA('NumElements',Nele,'ElementSpacing',lambda/2);
fs = 1.0e6;
prf = 10.0e3;               % 펄스 반복 주파수
height = 1000.0;            % 항공 레이더
direction = [90;0];
speed = 2.0e3;
depang = 30.0;
mountingAng = [depang,0,0];

% Create the clutter simulation object. The configuration assumes the earth is flat.
% The maximum clutter range of interest is 5 km, and the maximum azimuth coverage is ±60°.

Rmax = 5000.0;          % 클러터가 발생할 최대 거리
Azcov = 120.0;          % 클러터 발생 방위각 범위
tergamma = 0.0;         % 클러터 반사 강도의 감마 계수
tpower = 5000.0;        % 레이더 송신 전력
clutter_1 = constantGammaClutter('Sensor',array,...
    'PropagationSpeed',c,'OperatingFrequency',fc,'PRF',prf,...
    'SampleRate',fs,'Gamma',tergamma,'EarthModel','Flat',...
    'TransmitERP',tpower,'PlatformHeight',height,...
    'PlatformSpeed',speed,'PlatformDirection',direction,...
    'MountingAngles',mountingAng,'ClutterMaxRange',Rmax,...
    'ClutterAzimuthSpan',Azcov,'SeedSource','Property',...
    'Seed',40547);

% Simulate the clutter return for 10 pulses.

Nsamp = fs/prf;
Npulse = 10;
sig = zeros(Nsamp,Nele,Npulse);         % 100개의 행, 4개의 열, 10개의 차원으로 된 데이터 신호 데이터 생성
for m = 1:Npulse    % 각 펄스에 대해 클러터 신호를 생성
    sig(:,:,m) = clutter_1();
end

% Plot the angle-Doppler response of the clutter at the 20th range bin.

response = phased.AngleDopplerResponse('SensorArray',array,...
    'OperatingFrequency',fc,'PropagationSpeed',c,'PRF',prf);
plotResponse(response,shiftdim(sig(20,:,:)),'NormalizeDoppler',true)

surfclutterrcs

%% Range-Doppler 에서 시각화

% Reshape the signal matrix to match the required format
sig2D = reshape(sig, Nsamp, Nele * Npulse);

% Create the Range-Doppler response object without PRF
response = phased.RangeDopplerResponse('RangeMethod', 'FFT', 'DopplerOutput', 'Frequency', 'SampleRate', fs);

% Compute and plot the Range-Doppler map
plotResponse(response, sig2D);

%% Simulate Clutter Using Known Transmit Signal %%

% Simulate the clutter return from terrain with a gamma value of 0 dB. 
% You input the transmit signal of the radar system when creating clutter. 
% In this case, you do not use the TransmitERP property.

% Set up the characteristics of the radar system. This system has a 4-element uniform linear array (ULA). 
% The sample rate is 1 MHz, and the PRF is 10 kHz. The propagation speed is the speed of light,
% and the operating frequency is 300 MHz. The radar platform is flying 
% 1 km above the ground with a path parallel to the ground 
% along the array axis. The platform speed is 2 km/s. The mainlobe has a depression angle of 30°.

Nele = 4;
c = physconst('Lightspeed');
fc = 300.0e6;
lambda = c/fc;
ula = phased.ULA('NumElements',Nele,'ElementSpacing',lambda/2);
fs = 1.0e6;
prf = 10.0e3;
height = 1.0e3;
direction = [90;0];
speed = 2.0e3;
depang = 30;
mountingAng = [depang,0,0];

% Create the clutter simulation object and configure it to accept an transmit signal as an input argument. 
% The configuration assumes the earth is flat. The maximum clutter range of interest is 5 km, 
% and the maximum azimuth coverage is ±60°.

Rmax = 5000.0;
Azcov = 120.0;
tergamma = 0.0;
clutter_2 = constantGammaClutter('Sensor',ula,...
    'PropagationSpeed',c,'OperatingFrequency',fc,'PRF',prf,...
    'SampleRate',fs,'Gamma',tergamma,'EarthModel','Flat',...
    'TransmitSignalInputPort',true,'PlatformHeight',height,...
    'PlatformSpeed',speed,'PlatformDirection',direction,...
    'MountingAngles',mountingAng,'ClutterMaxRange',Rmax,...
    'ClutterAzimuthSpan',Azcov,'SeedSource','Property',...
    'Seed',40547);

% Simulate the clutter return for 10 pulses. At each step, pass the transmit signal as an input argument. 
% The software computes the effective transmitted power of the signal. The transmit signal is a 
% rectangular waveform with a pulse width of 2 μs.

tpower = 5.0e3;
pw = 2.0e-6;
X = tpower*ones(floor(pw*fs),1);
Nsamp = fs/prf;
Npulse = 10;
sig = zeros(Nsamp,Nele,Npulse);
for m = 1:Npulse
    sig(:,:,m) = step(clutter_2,X);
end

% Plot the angle-Doppler response of the clutter at the 20th range bin.

response = phased.AngleDopplerResponse('SensorArray',ula,...
    'OperatingFrequency',fc,'PropagationSpeed',c,'PRF',prf);
plotResponse(response,shiftdim(sig(20,:,:)),'NormalizeDoppler',true)
