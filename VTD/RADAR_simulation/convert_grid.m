function output = convert_grid(mode,input1,input2)

% Parameter Setting
D2R                    = pi/180;
R2D                    = 180/pi;
Radius_Earth           = 6378.137; % km
Grid                   = 0.09; % 격자간격(km)
Standard_Parallel_LAT1 = 30; % Lambert Conformal Conic 기준 투영위도
Standard_Parallel_LAT2 = 60;
% Origin_LAT             = 38.190860;  % 분석영역 기준점 위도
% Origin_LON             = 127.665801; % 분석영역 기준점 경도
Origin_LAT             = 38.330860;  % 분석영역 기준점 위도
Origin_LON             = 127.665801; % 분석영역 기준점 경도
Origin_X               = 1; %기준점의 X좌표
Origin_Y               = 1; %기준점의 Y좌표

% Lambert Conformal Conic 단위변환
Re = Radius_Earth / Grid;
Slat1 = Standard_Parallel_LAT1 * D2R;
Slat2 = Standard_Parallel_LAT2 * D2R;
Olat = Origin_LAT * D2R;
Olon = Origin_LON * D2R;

n = log(cos(Slat1)/cos(Slat2))/log(tan(pi*0.25+Slat2*0.5)/tan(pi*0.25+Slat1*0.5));
F = power(tan(pi*0.25+Slat1*0.5),n)*cos(Slat1)/n;
R0 = Re*F/power(tan(pi*0.25+Olat*0.5),n);



% 모드정의
% 1: 위경도 -> 좌표   v1=입력위도 v2=입력경도
% 2: 좌표 -> 위경도   v1=입력x좌표 v2=입력y좌표

% mode = 2; %모드 선택
% LAT_input = 35.4277;
% LON_input = 128.2164 ;
% X_input = 379;
% Y_input = 248;

if(mode == 1)
    v1 = input1;
    v2 = input2;
    Ra = Re*F/power(tan(pi*0.25 + v1*D2R*0.5),n);
    theta = v2 * D2R - Olon;
    if(theta>pi)
        theta = theta - 2*pi;
    end
    if(theta<-pi)
        theta = theta + 2*pi;
    end
    theta = theta*n;
    x = Ra*sin(theta) + Origin_X + 0.5;
    y = R0 - Ra*cos(theta) + Origin_Y + 0.5;
    output(1) = x;
    output(2) = y;
elseif(mode==2)
    v1 = input1;
    v2 = input2;
    Xn = v1 - Origin_X;
    Yn = R0 - v2 + Origin_Y;
    Ra = sqrt(Xn.^2 + Yn.^2);
    if(n<0)
        Ra = -Ra;
    end
    alat = 2*atan(power((Re*F/Ra),(1/n))) - pi*0.5;
    if(abs(Xn) <= 0)
        theta = 0;
    else
        if(abs(Yn) <= 0)
            theta = pi*0.5;
            if(Xn<0)
                theta = -theta;
            end
        else
            theta = atan2(Xn,Yn);
        end
        
    end
    alon = theta/n + Olon;
    LAT_output = alat*R2D;
    LON_output = alon*R2D;
    output(1) = LAT_output;
    output(2) = LON_output;
end

end






