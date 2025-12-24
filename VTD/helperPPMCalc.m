function helperPPMCalc()



% 테이블을 먼저 입력받음

clear_table = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/EO/tables/clear_sky.xlsx';
moderate_rain_table = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/EO/tables/moderate_rain.xlsx';
heavy_rain_table = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/EO/tables/heavy_rain.xlsx';
snow_table = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/EO/tables/snow.xlsx';
fog_table = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/EO/tables/fog.xlsx';
cloud_table = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/EO/tables/cloud.xlsx';
forest_table = 'C:/Users/leeyj/OneDrive - 인하대학교/school/assignment/vtd13/data/EO/tables/forest.xlsx';
sheet = 1;  % 첫 번째 시트

% 입력 파라미터의 날씨 및 배경에 따라 조건문을 거쳐 테이블을 선정
disp("weather conditions: clear, moderate rain, heavy rain, snow, fog, cloud, forest")
condition = input('Enter the weather conditio: ', 's');

if strcmpi(condition, 'clear')
    disp("맑은 날씨 상황이 선택되었습니다.");
    userTable = readmatrix(clear_table, 'Sheet', sheet);
elseif strcmpi(condition, 'moderate rain')
    disp("약간의 비가 오는 날씨 상황이 선택되었습니다.");
    userTable = readmatrix(moderate_rain_table, 'Sheet', sheet);
elseif strcmpi(condition, 'heavy rain')
    disp("거센 비가 오는 날씨 상황이 선택되었습니다.");
    userTable = readmatrix(heavy_rain_table, 'Sheet', sheet);
elseif strcmpi(condition, 'snow rain')
    disp("눈이 오는 날씨 상황이 선택되었습니다.");
    userTable = readmatrix(snow_table, 'Sheet', sheet);
elseif strcmpi(condition, 'fog')
    disp("안개가 낀 날씨 상황이 선택되었습니다.");
    userTable = readmatrix(fog_table, 'Sheet', sheet);
elseif strcmpi(condition, 'cloud')
    disp("구름이 낀 날씨 상황이 선택되었습니다.");
    userTable = readmatrix(cloud_table, 'Sheet', sheet);
elseif strcmpi(condition, 'forest')
    disp("숲 배경의 상황이 선택되었습니다.");
    userTable = readmatrix(forest_table, 'Sheet', sheet);
else
    disp('입력 값이 잘못되었습니다.');
end

% 수직이착륙기 기하(고각, 방위각)를 입력받음
ele = input('Enter elevation: ');
azi = input('Enter azimuth: ');
if azi <0   % 방위각이 음수일 경우 대칭성을 이용하여 양수의 범위에서 값을 찾음
    azi = abs(azi);
end    
azi = azi+2;
ele = ele+92;
% disp(['방위각:', num2str(azi), ' 선택됨, 고각:', num2str(ele),' 선택됨.']);

% 수직이착륙기와 레이더 간 거리를 입력받음
% 300미터부터 3000미터 까지 100미터 단위로 입력받음
dist = input("카메라로부터 수직이착륙기 까지의 거리를 입력하세요(300m부터 3000m까지 입력: ");
if dist < 300 && dist > 3000
    disp("잘못된 값을 입력하셨습니다.");
end
if dist < 300
    dist = 300;
end
if dist > 3000
    dist = 3000;
end

disp([num2str(dist), '의 거리가 입력되었습니다.']);

refPixel = 202;
minPixelCnt = 25;
originalPixel = userTable(ele,azi);

% 사전에 미리 구한 이중 지수 함수의 파라미터를 이용
a = 20276.7791;
b = -0.0075;
c = 1114.5824;
d = -0.0015;

double_exp_model = @(x) a * exp(b * x) + c * exp(d * x);    % 지수 함수 피팅 모델

calculated_pixel = double_exp_model(dist);  % 비율을 구하기 위해 사용자가 입력한 거리에서 구해진 픽셀 수
pixelRatio = calculated_pixel/refPixel;     % 두 변수를 통해 비율을 계산
finalPPM = pixelRatio * originalPixel;     % 구한 비율을 특정 기상 상황 및 특정 기하에서의 픽셀과 곱함

disp(['계산된 PPM 값: ', num2str(finalPPM)]);

if finalPPM > minPixelCnt
    disp("목표가 식별 됨");
else
    disp("목표 식별 불가")
end