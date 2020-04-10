close all;
clear;

%-----------------------------
% 产生信号
source;
tag;
y_sig = y_source .* y_tag;

figure
subplot(3, 1, 1);
plot(t / ratio, y_source);
subplot(3, 1, 2);
plot(t / ratio, y_tag);
subplot(3, 1, 3);
plot(t / ratio, y_sig)
% r_tagIn = 6.5, r_tagOut = 3.25, r_direct = 9.086
%---------------------------------------------------
% 衰减计算
P_source = 50;

% [L_out] = ideal_decline(fx, r, G_tx, G_rx)
% fx(MHz), r(m), G_tx, G_rx
% tag 之前
f_tx = f_source * ratio;
r_tagIn = 6.5;
G_source = 0;
G_tag = 0;
L_tagIn = ideal_decline(f_tx * 10^(-6), r_tagIn, G_source, G_tag);
P_tagIn = P_source * 10^(-0.1 * L_tagIn);

% 直射以及 tag 之后
r_tagOut = 3.25;
r_direct = 9.086;
G_rx = 0;
L_tagOut = ideal_decline(f_tx * 10^(-6), r_tagOut, G_tag, G_rx);
L_direct = ideal_decline(f_tx * 10^(-6), r_direct, G_source, G_rx);
P_tagOut = P_tagIn * 10^(-0.1 * L_tagOut);
P_direct = P_source * 10^(-0.1 * L_direct);

L_test = ideal_decline(f_tx * 10^(-6), r_tagIn + r_tagOut, G_source, G_rx);
P_test = P_source * 10^(-0.1 * L_test); 
%--------------------------------------
% 50 mW  的发射功率
decline_direct = y_source * (P_direct / P_source);
% decline_tag = y_sig * (P_tagOut / P_source);
decline_tag = y_sig * (P_test / P_source);
figure
subplot(2, 1, 1);
plot(t / ratio, decline_direct);
subplot(2, 1, 2);
plot(t / ratio, decline_tag);


%-------------------------------------

% 计算延时
c = 3 * (10 ^ 8);
delay_direct = r_direct / c ;
delay_reflect = (r_tagIn + r_tagOut) / c;
% 时间差，反射减去直射（非负）
delta_t = delay_reflect - delay_direct;
shift_reflect = zeros(1, round(delay_reflect * ratio * fs));
shift_direct = zeros(1, round(delay_direct * ratio * fs));
% 以反射时长为基准，扩展时间轴
n_plot = 0 : (length(shift_reflect) + N - 1);
t_plot = n_plot / fs;
% 画出延时后的波形
figure
subplot(3, 1, 1);
% plot(t / ratio, decline_direct);
plot(t_plot / ratio, [shift_direct decline_direct zeros(1, length(shift_reflect) - length(shift_direct))])
subplot(3, 1, 2);
% plot(t / ratio, decline_tag);
plot(t_plot / ratio, [shift_reflect decline_tag])
subplot(3, 1, 3);
% plot(t / ratio, decline_tag);
plot(t_plot / ratio, [shift_direct decline_direct zeros(1, length(shift_reflect) - length(shift_direct))] + [shift_reflect decline_tag])
%--------------------------------------



% clear;
