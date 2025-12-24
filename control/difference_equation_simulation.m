G = 0.5;
X = 1.0;
TS = 0.1;
Y = 0.0;
T = 0.0;
count = 0;

for N = 1:20
    Y = Y + G * (X - Y);
    T = N * TS;
    YTHEORY = 1 - (1 - G)^N;
    count = count + 1;
    ArrayT(count) = T;
    ArrayY(count) = Y;
    ArrayYTHEORY(count) = YTHEORY;
end

figure
plot(ArrayT, ArrayY, ArrayT, ArrayYTHEORY);
grid on;
title('Output');
xlabel('T (S)');
ylabel('Y');

clc
output = [ArrayT', ArrayY', ArrayYTHEORY'];
save('datfil', 'output', '-ascii');
disp(['Simulation finished' ...
    '']);