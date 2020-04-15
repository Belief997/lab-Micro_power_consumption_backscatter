% clear;
config;

% 采样率 10 倍
global fs;
fs = f_source * 10;

fx = f_source/10;

A = 1000000;
E = 2 * A;
%采样点数 10s
N = 10 * fs;
n = 0 : N-1;
%时间序列
% global t;
t = n / fs ;
global y_source;
y_source = A * sin(2 * pi * fx * t) + E;

[b,a]=sos2tf(SOS,G);
y_fil=filter(b,a,y_source);

figure
subplot(3,1,1);
plot(t / ratio, y_source);

subplot(3,1,2)
plot(y_fil);

subplot(3,1,3)
% disp(length(t));
% plot(y_fil(1:round(length(y_fil))/2));
plot(y_fil(1:1000));







