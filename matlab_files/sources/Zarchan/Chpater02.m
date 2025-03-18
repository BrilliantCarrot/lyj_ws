%% SIMULATION OF SECOND-ORDER SYSTEM

T=0.;
S=0.;
Y=0.;
YD=0.;
X=1.;
H=.001;
n=0.;
while T <=(1.-1e-5)
    YOLD=Y;
    YDOLD=YD;
    STEP=1;
    FLAG=0;
        while STEP <=1
            if FLAG==1
            STEP=2;
            Y=Y+H*YD;
            YD=YD+H*YDD;
            T=T+H;
            end
        YDD=W*X-W*W*Y;
        FLAG=1;
        end
    FLAG=0;
    Y=.5*(YOLD+Y+H*YD);
    YD=.5*(YDOLD+YD+H*YDD);
    S=S+H;
    if S >=.000999
        S=0.;
        n=n+1;
        ArrayT(n)=T;
        ArrayY(n)=Y;
    end
end
figure
plot(ArrayT,ArrayY),grid
xlabel('Time (Sec)')
ylabel('y')
clc
output=[Array',ArrayY'];
save datfil output -ascii
disp ’simulation finished’

%%

