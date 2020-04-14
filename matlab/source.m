% 采样率 10 倍
global fs;
fs = f_source * 10;
A = 1;
E = 2 * A;
%采样点数 10s
N = 10 * fs;
n = 0 : N-1;
%时间序列
% global t;
t = n / fs ;
global y_source;
y_source = A * sin(2 * pi * f_source * t) + E;
%画出信号源
% figure
% plot(t / ratio, y_source);

% 保存信号
global dirname;
filename = sprintf('%s%s', dirname, 'y_source');
save(filename, 'y_source');




