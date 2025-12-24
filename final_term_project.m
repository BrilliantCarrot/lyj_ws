clc; 
clear;
close all;
format long
mu = 398600.400;

r_c_t0 = [6900 0 0]';               % [km] chaser rad
v_c_t0 = [0 7.600538 0]';           % [km/s] chaser vel
r_c_norm = norm(r_c_t0);            % [km] chaser rad norm
r_t_norm = 7000;                    % [km] target rad
R = r_t_norm / r_c_norm;            % [-] if R<11.94, Hohmann transfer efficient
n_c = sqrt( mu / r_c_norm^3 );      % [1/s] chaser mean motion
n_t = sqrt( mu / r_t_norm^3 );      % [1/s] target mean motion
time_c = 2 * pi / n_c;              % [s] chaser cycle of revolution T
time_c_hr = time_c / 3600;          % [hr]
time_t = 2 * pi / n_t;              % [s] target cycle of revolution T
time_t_hr = time_t / 3600;          % [hr]

[a_c, e_c, i_c, RAAN_c, w_c, f_c] = ijk2keplerian(r_c_t0, v_c_t0);
% [km,-,deg,deg,deg,]
Chaser = [r_c_norm, 0, 0, 0, 0, f_c];           
Target = [r_t_norm, 0, 0, 0, 0, 5];                  
% Question 1.2
Target2 = [r_t_norm, 0, 0, 0, 0, -10];               
% Question 1.3
[r_t_t0, v_t_t0, test] = oe2eci(mu, Target);
[r_t2_t0, v_t2_t0, test] = oe2eci(mu, Target2);
%% Question 1.1 (Hohmann transfer)

a1 = (r_c_norm + r_t_norm) / 2;
del_v1 = sqrt( 2*mu / r_c_norm - mu / a1 ) - sqrt( mu / r_c_norm );
del_v2 = sqrt( mu / r_t_norm) - sqrt( 2*mu / r_t_norm - mu / a1 );
del_v = del_v1 + del_v2;

del_v1*1000;
del_v2*1000;
del_v*1000;

%% Question 1.2

time_trans = pi * sqrt( a1^3 / mu );
time_trans_min = time_trans / 60;

alpha_t = n_t * time_trans; 
del_f = pi - alpha_t;
del_f_deg = rad2deg(del_f);

del_f_2 = deg2rad(5);
Nrev = 0;

time_wait = ( del_f_2 - del_f + 2*pi*Nrev ) / ( n_c - n_t );
time_wait_min = time_wait/60;

X_c_0 = [r_c_t0; v_c_t0];
X_t_0 = [r_t_t0; v_t_t0];

dt = 1;
tspan_0 = 0 : dt : round(time_wait) ;
[time_c_dist, X_c_dist] = ode89( @f_orbit, tspan_0, X_c_0 );
[time_t_dist, X_t_dist] = ode89( @f_orbit, tspan_0, X_t_0 );

X_c_1 = X_c_dist(end, :);
X_t_1 = X_t_dist(end, :);
f_c_1 = atan( X_c_1(2) / X_c_1(1) );
f_c_1_deg = rad2deg( f_c_1 );

X_c_1_v = X_c_1 + [ 0, 0, 0, del_v1 * cos(pi/2-f_c_1), -del_v1 * sin( pi / 2 - f_c_1), 0];

tspan_1 = time_wait : dt : time_wait + time_trans ;
[time_c_dist2, X_c_dist2] = ode89 (@f_orbit, tspan_1, X_c_1_v);
[time_t_dist2, X_t_dist2] = ode89 (@f_orbit, tspan_1, X_t_1);

d_position_1 = zeros(1,max(size(tspan_0)));
d_position_2 = zeros(1,max(size(tspan_1)));
d_f_1 = zeros(1,max(size(tspan_0)));
d_f_2 = zeros(1,max(size(tspan_1)));

anomaly_c_1 = atan2(X_c_dist(:,2),X_c_dist(:,1));
anomaly_t_1 = atan2(X_t_dist(:,2),X_t_dist(:,1));

for i = 1:max(size(tspan_0))
    d_position_1(i) = sqrt((X_c_dist(i,1)-X_t_dist(i,1))^2+(X_c_dist(i,2)-X_t_dist(i,2))^2);
    i=i+1;
end

for j = 1:max(size(tspan_1))
    d_position_2(j) = sqrt((X_c_dist2(j,1)-X_t_dist2(j,1))^2+(X_c_dist2(j,2)-X_t_dist2(j,2))^2);

    if atan2(X_c_dist2(j,2),X_c_dist2(j,1))<0
        anomaly_c_2(j) = 2*pi + atan2(X_c_dist2(j,2),X_c_dist2(j,1));
    else
        anomaly_c_2(j) = atan2(X_c_dist2(j,2),X_c_dist2(j,1));
    end

    if atan2(X_t_dist2(j,2),X_t_dist2(j,1))<0
        anomaly_t_2(j) = 2*pi + atan2(X_t_dist2(j,2),X_t_dist2(j,1));
    else
        anomaly_t_2(j) = atan2(X_t_dist2(j,2),X_t_dist2(j,1));
    end
    j = j + 1;
end

d_position = [d_position_1, d_position_2];
time = [tspan_0, tspan_1];

d_f = rad2deg([anomaly_c_1; anomaly_c_2']);
d_f2 = rad2deg([anomaly_t_1; anomaly_t_2']);

%% figure 
figure(1)
hold on
grid on
xlim([-8000 8000])
ylim([-8000 8000])
axis equal
p1 = plot(X_c_dist(1,1), X_c_dist(1,2),'rp');
p2 = plot(X_t_dist(1,1), X_t_dist(1,2), 'bo');
p3 = plot(X_c_dist2(end,1), X_c_dist2(end,2),'rp');
p4 = plot(X_t_dist2(end,1), X_t_dist2(end,2),'bo');
p5 = plot(X_c_dist(:,1), X_c_dist(:,2),'r-.');
p6 = plot(X_t_dist(:,1), X_t_dist(:,2), 'b-.');
p7 = plot(X_c_dist2(:,1), X_c_dist2(:,2),'r-');
p8 = plot(X_t_dist2(:,1), X_t_dist2(:,2), 'b-');

title('2차원 위성 궤적')
xlabel('X [km]')
ylabel('Y [km]')
legend([p1 p2 p5 p6 p7 p8],{'Chaser','Target','Chaser Initial Coasting','Target Initial Coasting',...
    'Chaser Trans','Target Trans'},'Position',[0.47 0.42 0.1 0.2]);

figure(2)
hold on
grid on
plot(time, d_position)
title('위성 간 상대 거리')
xlabel('t [sec]')
ylabel('dr [km]')

figure(3)
hold on
grid on
plot(time,d_f2-d_f)
title('위상차')
xlabel('t [sec]')
ylabel('df [deg]')

%% Queston 1.3

t  = 0:1:5704;
f_t = n_t.*t;
f_t_f = (deg2rad(-10) + f_t)';
f_t_f_d = rad2deg(f_t_f);

e_1 = ((1-cos(f_t_f)) ./ (r_c_norm / r_t_norm - cos(f_t_f)) - 1);
a_1 = r_c_norm ./ (1 - e_1);
del_v11 = sqrt( 2*mu/r_c_norm -mu./a_1 )-sqrt( mu./r_c_norm );
v_c_t2 = sqrt( (2*mu/r_t_norm - 2*mu/r_c_norm ) + (norm(v_c_t0) + del_v11).^2);

phi_t = atan(e_1.*sin(f_t_f)./(1+e_1.*cos(f_t_f)));

del_v22 = sqrt((v_c_t2.^2 + norm(v_t_t0)^2 - 2.*(norm(v_c_t0)).*v_c_t2.*cos(phi_t)));
del_v_sum = del_v11 + del_v22;

figure(4)
hold on
grid on
plot(t, del_v_sum)
xlabel('time [s]')
ylabel('del v sum [km/s]')

%% 문제 2번 유형 기본 궤적 생성

% clc;
% clear;
% close all;
format long
mu = 398600.4418;
a = 6900;
n = sqrt(mu/a^3);
tmax = 2*pi/n;
t = 0 : tmax/100 :tmax;
r_0 = [0, -300, 0]';
r_1 = [0, 300, 0]';
r_2 = [0, 30, 0]';
f_dot = sqrt( mu * norm(r_0) ) / norm(r_0)^2; 
T = 2*pi/f_dot;
theta = 0 : (2*pi/100) : 2*pi;
theta2 = -15 :30/20 : 15;
kos_x = 200 * cos(theta);
kos_y = 200 * sin(theta);
ac_x = [0, 200*cosd(theta2), 0];
ac_y = [0, 200*sind(theta2), 0];

figure(1)
hold on
grid on
plot(kos_x, kos_y,'color',"#EDB120",'LineStyle','--')
axis equal
plot(ac_x,ac_y,'LineStyle','--')
plot(r_0(2),r_0(1),'k.','MarkerSize',20)
text(r_0(2),r_0(1)+20,'r0')
plot(r_1(2),r_1(1),'k.','MarkerSize',20)
text(r_1(2),r_1(1)+20,'r1')
plot(r_2(2),r_2(1),'k.','MarkerSize',20)
text(r_2(2),r_2(1)+20,'r2')
legend('KOS', 'AC')
xlabel('Y [m]')
ylabel('X [m]')

%% Question 2.1

nt = 0 : 2*pi/100 : 2*pi;
x = @(nt) 200 / pi * cos(nt) - 200 / pi;
y = @(nt) -400 / pi * sin(nt) + 300 / pi * nt -300;
x_21 = x(nt);
y_21 = y(nt);
plot(y_21, x_21)

%% Question 2.2

p_inter = [-300 0 0]';
plot(p_inter(2),p_inter(1),'k.','MarkerSize',20)
text(p_inter(2),p_inter(1)+50,'Interim Point')
phi_rr = @(nt) [4-3*cos(nt), 0, 0; 6*(sin(nt)-nt), 1, 0; 0, 0, cos(nt)];
phi_rv = @(nt) [sin(nt), 2*(1-cos(nt)), 0; 
2*(cos(nt)-1), 1*(4*sin(nt)-3*nt), 0; 0, 0, sin(nt)];
phi_vr = @(nt) [3*sin(nt), 0, 0; 6*cos(nt)-6, 0, 0; 
0, 0, -sin(nt)];
phi_vv = @(nt) [cos(nt), 2*sin(nt), 0; -2*sin(nt), 4*cos(nt)-3, 0; 0, 0, cos(nt)];

dv1 = phi_rv(pi)^(-1)*(p_inter- phi_rr(pi)*r_0);
dv2 = -phi_vr(pi)*r_0- phi_vv(pi) * dv1;
dv3 = phi_rv(pi)^(-1)*(r_1-phi_rr(pi)*p_inter);
dv4 = -phi_vr(pi)*p_inter-phi_vv(pi)*dv3;

data =[];
x = [r_0; dv1];
for t=0:pi/100:pi
   STM = [phi_rr(t) phi_rv(t) ; phi_vr(t) phi_vv(t)];
   x_next = STM * x;
   data = [data x_next(1:3)];
end
x = [p_inter; dv3];

for t=0:pi/100:pi
   STM = [phi_rr(t) phi_rv(t) ; phi_vr(t) phi_vv(t)];
   x_next = STM * x;
   data = [data x_next(1:3)];
end

plot(data(2,:),data(1,:))

%% Question 2.3

t = 0 : 2*pi/100 : 2*pi;
x = @(nt) -90 / pi * cos(nt) + 90 / pi;
y = @(nt) 180 / pi * sin(nt) - 135 / pi * nt +300;
x_23 = x(nt);
y_23 = y(nt);
plot(y_23, x_23)

%% Question 2.4

p_inter1 = [0 135 0]';
p_inter2 = [0 80 0]';
plot(p_inter1(2),p_inter1(1),'k*')
plot(p_inter2(2),p_inter2(1),'k*')
phi_rr = @(nt) [4-3*cos(nt), 0, 0; 6*(sin(nt)-nt), 1, 0; 0, 0, cos(nt)];
phi_rv = @(nt) [sin(nt), 2*(1-cos(nt)), 0; 
                2*(cos(nt)-1), 1*(4*sin(nt)-3*nt), 0;
                0, 0, sin(nt)];
phi_vr = @(nt) [3*sin(nt), 0, 0; 
                6*cos(nt)-6, 0, 0; 
                0, 0, -sin(nt)];
phi_vv = @(nt) [cos(nt), 2*sin(nt), 0;
                -2*sin(nt), 4*cos(nt)-3, 0;
                0, 0, cos(nt)];
dv1 = phi_rv(pi)^(-1)*(p_inter1- phi_rr(pi)*r_1);
dv2 = phi_rv(pi)^(-1)*(p_inter2- phi_rr(pi)*p_inter1);
dv3 = phi_rv(pi)^(-1)*(r_2- phi_rr(pi)*p_inter2);
data =[];
x = [r_1; dv1];
for t=0:pi/100:pi
   STM = [phi_rr(t) phi_rv(t) ;
          phi_vr(t) phi_vv(t)];
   x_next = STM * x;
   data = [data x_next(1:3)];
end

x = [p_inter1; dv2];

for t=0:pi/100:pi
   STM = [phi_rr(t) phi_rv(t) ;
          phi_vr(t) phi_vv(t)];
   x_next = STM * x;
   data = [data x_next(1:3)];
end

x = [p_inter2; dv3];

for t=0:pi/100:pi
   STM = [phi_rr(t) phi_rv(t) ;
           phi_vr(t) phi_vv(t)];
   x_next = STM * x;
   data = [data x_next(1:3)];
end
plot(data(2,:),data(1,:))

%% functions

function Xdot = f_orbit(t,X)
    mu = 398600.4418;
    r = X(1:3);
    v = X(4:6);
    r_size = norm(r);
    c = -mu / r_size^3;
    dr = v;
    dv = c.*r;
    Xdot = [dr; dv];
end

function [r, v, test]=oe2eci(mu, oe)
    r = zeros(3,1);
    v = zeros(3,1);
    a = oe(1);
    e = oe(2);
    i = oe(3);
    w = oe(4);
    Omg = oe(5);
    f = oe(6);
    p = a*(1-e^2);
    theta = w+f;
    r_size = p/(1+e*cosd(f));
    h = sqrt(mu*p);
    r = r_size*[cosd(Omg)*cosd(theta)-sind(Omg)*sind(theta)*cosd(i);
        sind(Omg)*cosd(theta)+cosd(Omg)*sind(theta)*cosd(i);
        sind(theta)*sind(i)];
    v = -mu/h*[cosd(Omg)*(sind(theta)+e*sind(w))+sind(Omg)*(cosd(theta)+e*cosd(w))*cosd(i);
        sind(Omg)*(sind(theta)+e*sind(w))-cosd(Omg)*(cosd(theta)+e*cosd(w))*cosd(i);
        -(cosd(theta)+e*cosd(w))*sind(i)];
    test=r_size/norm(r);
end



