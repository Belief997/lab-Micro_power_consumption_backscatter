clc; clear;
% Ƶ�� 2.412 * 10k (G) 
ratio = 10 ^ 5;
f_source = 2.412 * (10 ^ 4);
% ������ 1M
fs = f_source * 10;
A = 1;
E = 2 * A;
%�������� 10s
N = 10 * fs;
n = 0 : N-1;
%ʱ������
t = n / fs ;
y_source = A * sin(2 * pi * f_source * t) + E;
%�����ź�Դ
% figure
% plot(t / ratio, y_source);




