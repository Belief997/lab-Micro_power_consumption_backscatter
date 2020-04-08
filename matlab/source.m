clc; clear;
% 频率 10k
f1 = 10000;
% 采样率 1M
fs = 1000000;
%采样点数 10s
N = 10 * fs;
n = 0 : N-1;
%时间序列
t = n / fs;
y = sin(2 * pi * f1 * t);
%画出信号源
plot(t, y);




