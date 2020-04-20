% clear;
% config;

% ������ 10 �� 24k
f_source = 2.412 * (10 ^ 4);
fs = f_source * 10;

fx = 1;

A = 1000;
E = 2 * A;
%�������� 10s
N = 10 * fs;
n = 0 : N-1;
%ʱ������
% global t;
t = n / fs ;

y_1 = A * sin(2 * pi * f_source * t) + E;
% y_2 = A * sin(2 * pi * fx * t) + E;
% y_2 = A * square(2 * pi * fx * t, 50) + E;
y_2 =  square(2 * pi * fx * t, 50) + 1;
y_3 = y_1 .* y_2 ;

figure
subplot(3,1,1);
plot(y_1);

subplot(3,1,2)
plot(y_2);

subplot(3,1,3)
plot(y_3);

% wp: 10k, ws: 20k
[b,a]=sos2tf(SOS,G);
y_fil=filter(b,a, y_3);

figure
subplot(3,1,1);
plot(y_3);

subplot(3,1,2)
plot(y_fil);

% subplot(3,1,3)
% disp(length(t));
% plot(y_fil(1:round(length(y_fil))/2));
% plot(y_fil(1:1000));






