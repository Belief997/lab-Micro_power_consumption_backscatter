% 全局变量和常量

% 光速
global c;
c = 3 * (10 ^ 8);

% 房间尺寸 5m x 6m
global Width;
global Length;
global block_Nx;
global block_Ny;
Width = 5;
Length = 6;
block_Nx = 2;
block_Ny = 2;

global H1;
global H2;
global H0;
H0 = 1.25;
H1 = 1.25;
H2 = 0.75;

global D3;
D3 = sqrt(Length ^2 + (H1 - H2 ^2));

% 仿真比率，发射源信号频率 2.412 * 10k (G) 
global ratio;
ratio = 10 ^ 5;
global f_source;
f_source = 2.412 * (10 ^ 4);

% 信号发射功率 mW
global P_source;
P_source = 50;

global dirname;
time_now = datestr(now, 30);
dirname = sprintf('.\\%s\\', time_now);
mkdir(dirname);

