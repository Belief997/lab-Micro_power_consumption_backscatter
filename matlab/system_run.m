close all;
clear;

% ����
config;
%-----------------------------
% �����ź�
source;
tag;
global y_sig;
y_sig = y_source .* y_tag;

%% ��Ӹ�˹����
% y_noise = awgn(y_sig, 50);
% figure
% subplot(3, 1, 1);
% plot(t / ratio, y_sig);
% subplot(3, 1, 2);
% plot(t / ratio, y_noise);
% subplot(3, 1, 3);
% plot(t / ratio, y_noise - y_sig)


%% �����ź�
global dirname;
filename = sprintf('%s%s', dirname, 'y_sig');
save(filename, 'y_sig');

figure
subplot(3, 1, 1);
plot(t / ratio, y_source);
subplot(3, 1, 2);
plot(t / ratio, y_tag);
subplot(3, 1, 3);
plot(t / ratio, y_sig)

disp(f_source);

%% ------------------------------------------
% ��������
global Width;
global Length;
rx = Width / (2 * block_Nx);
ry = Length / (2 * block_Ny);
temp = (1 : block_Nx);
x = ones(block_Ny, block_Nx);
x = x .* temp;

temp = (1 : block_Ny);
y = ones(block_Ny, block_Nx);
y = y .* temp';

x = (2 * x - ones(size(x))) * rx;
y = (2 * y - ones(size(x))) * ry;

% ������������
[d1, d2, d3] = xy2d(x, y);

% ---------------------------------------
% single_run(r_tagIn,r_tagOut, r_direct, draw_plot, )
single_run(d1, d2, d3, false, 'tag');


%% ---------------------------------------
% ���� fence �ź�
r_bx = 5 / 3;
r_by = 6 / 4;
x_b = [0 0 0;3 3 3;1 2 3;1 2 3;];
y_b = [1 2 3;1 2 3;0 0 0;4 4 4;];

x_b = x_b * r_bx;
y_b = y_b * r_by;

% ������������
% h1_b = H1 * ones(size(x_b));
% h2_b = H2 * ones(size(x_b));
% d1_b = sqrt(x_b .^2 + y_b .^2 + h1_b .^2);
% d2_b = sqrt(x_b .^2 + (Length * ones(size(x_b)) - y_b) .^2 + h2_b .^2);
% d3_b = D3 * ones(size(x_b));

[d1_b, d2_b, d3_b] = xy2d(x_b, y_b);
% single_run(r_tagIn,r_tagOut, r_direct, draw_plot)
single_run(d1_b, d2_b, d3_b, false, 'beacon');

%%
% ����Ԥ����
global block_Nx;
global block_Ny;
% [] = preproc(filename, i, draw, isBeacon)
for i=1: block_Nx * block_Ny
    preproc('sig_rx', i, false, false);
end

% beacon
for i=1: 12
    preproc('Beacon_sig_rx', i, false, true);
end

% % r_tagIn = 6.5, r_tagOut = 3.25, r_direct = 9.086
%% ---------------------------------------------------
% % ˥������
% 
% % [L_out] = ideal_decline(fx, r, G_tx, G_rx)
% % fx(MHz), r(m), G_tx, G_rx
% % tag ֮ǰ
% f_tx = f_source * ratio;
% r_tagIn = 6.5;
% G_source = 0;
% G_tag = 0;
% L_tagIn = ideal_decline(f_tx * 10^(-6), r_tagIn, G_source, G_tag);
% P_tagIn = P_source * 10^(-0.1 * L_tagIn);
% 
% % ֱ���Լ� tag ֮��
% r_tagOut = 3.25;
% r_direct = 9.086;
% G_rx = 0;
% L_tagOut = ideal_decline(f_tx * 10^(-6), r_tagOut, G_tag, G_rx);
% L_direct = ideal_decline(f_tx * 10^(-6), r_direct, G_source, G_rx);
% P_tagOut = P_tagIn * 10^(-0.1 * L_tagOut);
% P_direct = P_source * 10^(-0.1 * L_direct);
% 
% % ʹ�� tag ǰ��ľ���ͼ���˥����������
% % L_test = ideal_decline(f_tx * 10^(-6), r_tagIn + r_tagOut, G_source, G_rx);
% % P_test = P_source * 10^(-0.1 * L_test); 
% %--------------------------------------
% 
% decline_direct = y_source * (P_direct / P_source);
% decline_tag = y_sig * (P_tagOut / P_source);
% % decline_tag = y_sig * (P_test / P_source);
% figure
% subplot(2, 1, 1);
% plot(t / ratio, decline_direct);
% subplot(2, 1, 2);
% plot(t / ratio, decline_tag);
% 
% 
% %-------------------------------------
% 
% % ������ʱ
% c = 3 * (10 ^ 8);
% delay_direct = r_direct / c ;
% delay_reflect = (r_tagIn + r_tagOut) / c;
% % ʱ�������ȥֱ�䣨�Ǹ���
% delta_t = delay_reflect - delay_direct;
% shift_reflect = zeros(1, round(delay_reflect * ratio * fs));
% shift_direct = zeros(1, round(delay_direct * ratio * fs));
% % �Է���ʱ��Ϊ��׼����չʱ����
% n_plot = 0 : (length(shift_reflect) + N - 1);
% t_plot = n_plot / fs;
% % ������ʱ��Ĳ���
% figure
% subplot(3, 1, 1);
% % plot(t / ratio, decline_direct);
% plot(t_plot / ratio, [shift_direct decline_direct zeros(1, length(shift_reflect) - length(shift_direct))])
% subplot(3, 1, 2);
% % plot(t / ratio, decline_tag);
% plot(t_plot / ratio, [shift_reflect decline_tag])
% subplot(3, 1, 3);
% % plot(t / ratio, decline_tag);
% plot(t_plot / ratio, [shift_direct decline_direct zeros(1, length(shift_reflect) - length(shift_direct))] + [shift_reflect decline_tag])
% %--------------------------------------



% clear;
