% 全局变量和常量

% 光速
global c;
c = 3 * (10 ^ 8);

% 房间尺寸 5m x 6m
global Width;
global Length;
global Hight;
global block_Nx;
global block_Ny;
Width = 5;
Length = 6;
Hight = 3;
block_Nx = 5;
block_Ny = 6 ;

global H0;
global H1;
global H2;
global H3;
H0 = 1.25; % 整体的偏移
H1 = 1.25; % 发射器 2.5m
H2 = 0.75; % 接收器_1 2m 
H3 = 0.85; % 接收器_2 2.1m

global D3;
D3 = sqrt(Length ^2 + (H1 - H2 ^2));

% 仿真比率，发射源信号频率 2.412 * 10k (G) 
global ratio;
ratio = 10 ^ 5;
global f_source;
f_source = 2.412 * (10 ^ 4);

% 信号发射功率 mW
global P_source;
P_source = 100;
% 噪声功率 dBW
global P_noise;
P_noise = 0.5;
% 接收灵敏度及其接收功率阈值
global L_recTre;
RecSen = -100; % dBm
P_recTre = 10 ^ (RecSen / 10);
L_recTre = 10 * log10(P_source / P_recTre)
% 信噪比
global SNR;
SNR = 250;


global dirname;
time_now = datestr(now, 30);
dirname = sprintf('.\\%s\\', time_now);

global run_type;
if run_type ~= 'io'
    mkdir(dirname);
    % 保存配置信息, 将会覆盖已有的配置文件
    filename = sprintf('.\\%s', 'config');
    save(filename, 'Width', 'Length', 'block_Nx', 'block_Ny', 'dirname');
end
