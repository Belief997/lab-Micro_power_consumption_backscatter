
% ������ 1M
global fs;
fs = f_source * 10;
A = 1;
E = 2 * A;
%�������� 10s
N = 10 * fs;
n = 0 : N-1;
%ʱ������
% global t;
t = n / fs ;
global y_source;
y_source = A * sin(2 * pi * f_source * t) + E;
%�����ź�Դ
% figure
% plot(t / ratio, y_source);




