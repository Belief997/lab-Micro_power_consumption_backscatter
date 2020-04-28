close all;
clear;

% 配置
global run_type;
run_type = 'bt';
config;
%-----------------------------
%% 信号源
source;
tag;
global y_sig;
y_sig = y_source .* y_tag;

% 添加高斯噪声
% dB 单位信噪比
% y_noise = awgn(y_sig, 50);  % 输入复信号，则加复噪声
% y_sig = y_noise;
% figure
% subplot(3, 1, 1);
% plot(t / ratio, y_sig);
% subplot(3, 1, 2);
% plot(t / ratio, y_noise);
% subplot(3, 1, 3);
% plot(t / ratio, y_noise - y_sig)


% 保存信号
global dirname;
filename = sprintf('%s%s', dirname, 'y_sig');
save(filename, 'y_sig');

% figure
% subplot(3, 1, 1);
% plot(t / ratio, y_source);
% subplot(3, 1, 2);
% plot(t / ratio, y_tag);
% subplot(3, 1, 3);
% plot(t / ratio, y_sig)
% 
% disp(f_source);

%% 计算 block 坐标
% 
global Width;
global Length;
global block_Nx;
global block_Ny;
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

%% 设定 fence 位置
% 
r_bx = 5 / 3;
r_by = 6 / 4;
x_b = [0 0 0;3 3 3;1 2 3;1 2 3;];
y_b = [1 2 3;1 2 3;0 0 0;4 4 4;];

x_b = x_b * r_bx;
y_b = y_b * r_by;


%% 信道
% 计算三个距离
[d1, d2, d3] = xy2d(x, y, 1);

% single_run(r_tagIn,r_tagOut, r_direct, draw_plot, )
single_run(d1, d2, d3, false, 'tag');

[d1_b, d2_b, d3_b] = xy2d(x_b, y_b, 1);
% single_run(r_tagIn,r_tagOut, r_direct, draw_plot)
single_run(d1_b, d2_b, d3_b, false, 'beacon');



%% 接收端
% 数据处理
global block_Nx;
global block_Ny;
% [] = preproc(filename, i, draw, isBeacon)
for i=1: block_Nx * block_Ny
    preproc('sig_rx', i, false, 'tag');
end

% beacon
for i=1: 12
    preproc('Beacon_sig_rx', i, false, 'beacon');
end

% clear;
