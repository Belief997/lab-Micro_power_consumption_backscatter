% ȫ�ֱ����ͳ���

% ����
global c;
c = 3 * (10 ^ 8);

% ����ߴ� 5m x 6m
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
H0 = 1.25; % �����ƫ��
H1 = 1.25; % ������ 2.5m
H2 = 0.75; % ������_1 2m 
H3 = 0.85; % ������_2 2.1m

global D3;
D3 = sqrt(Length ^2 + (H1 - H2 ^2));

% ������ʣ�����Դ�ź�Ƶ�� 2.412 * 10k (G) 
global ratio;
ratio = 10 ^ 5;
global f_source;
f_source = 2.412 * (10 ^ 4);

% �źŷ��书�� mW
global P_source;
P_source = 100;
% �������� dBW
global P_noise;
P_noise = 0.5;
% ���������ȼ�����չ�����ֵ
global L_recTre;
RecSen = -100; % dBm
P_recTre = 10 ^ (RecSen / 10);
L_recTre = 10 * log10(P_source / P_recTre)
% �����
global SNR;
SNR = 250;


global dirname;
time_now = datestr(now, 30);
dirname = sprintf('.\\%s\\', time_now);

global run_type;
if run_type ~= 'io'
    mkdir(dirname);
    % ����������Ϣ, ���Ḳ�����е������ļ�
    filename = sprintf('.\\%s', 'config');
    save(filename, 'Width', 'Length', 'block_Nx', 'block_Ny', 'dirname');
end
