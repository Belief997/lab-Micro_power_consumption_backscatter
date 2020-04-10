clc; clear;
% 频率 2.412 * 10k (G) 
ratio = 10 ^ 5;
f_source = 2.412 * (10 ^ 4);
% 采样率 1M
fs = f_source * 10;
A = 1;
E = 2 * A;
%采样点数 10s
N = 10 * fs;
n = 0 : N-1;
%时间序列
t = n / fs ;
y_source = A * sin(2 * pi * f_source * t) + E;
%画出信号源
% figure
% plot(t / ratio, y_source);




