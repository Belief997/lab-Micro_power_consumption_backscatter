% ȫ�ֱ����ͳ���

% ����
global c;
c = 3 * (10 ^ 8);

% ����ߴ� 5m x 6m
global width;
global length;
global block_Nx;
global block_Ny;
width = 5;
length = 6;
block_Nx = 2;
block_Ny = 2;

global H1;
global H2;
global H0;
H0 = 1.25;
H1 = 1.25;
H2 = 0.75;

global D3;
D3 = sqrt(length ^2 + (H1 - H2 ^2));

% ������ʣ�����Դ�ź�Ƶ�� 2.412 * 10k (G) 
global ratio;
ratio = 10 ^ 5;
global f_source;
f_source = 2.412 * (10 ^ 4);

% �źŷ��书�� mW
global P_source;
P_source = 50;

global dirname;
time_now = datestr(now, 30);
dirname = sprintf('.\\%s\\', time_now);
mkdir(dirname);

