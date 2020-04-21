function [] = preproc(filename, i, draw, type)
%PREPROC 对接收的信号做预处理，去载波，滤波
%   

global dirname;
savePath = dirname;
% xxx -> xxx_i -> .\xxx\xxx_i
if strcmp(type ,  'inside') || strcmp(type ,  'outside')
    savePath = '.\DataSet\';
end
file_name = sprintf('%s%s_%d.mat',savePath, filename, i);

% 为了保持同频同相
sig_dir = cell2mat(struct2cell(load(file_name, 'sig_dir')));
sig_rx = cell2mat(struct2cell(load(file_name, 'sig_rx')));

% y_u = y_0 .* y_t;
% y_u = y_t;
% 下变频
y_conv =  sig_dir .* sig_rx;


if draw
    figure;
    subplot(2,2,1);
    plot(sig_rx);
    subplot(2,2,2);
    plot(y_conv);    
end

% 滤波
% 10 k, 20k,  12dB, 80dB
load('G.mat');
load('SOS.mat');
% FIR: b=Num,a=1;
[b,a]=sos2tf(SOS,G);
y_fil=filter(b,a, y_conv);

% 截取
y_cut = y_fil(1,1000:2400000);
y_smooth = smooth(y_cut, 800);

% 保存数据

if strcmp(type ,  'beacon')
    savename = sprintf('%sBeacon_%s_%d', savePath, 'sig_proc', i);
elseif strcmp(type , 'tag')
    savename = sprintf('%s%s_%d', savePath, 'sig_proc', i);  
elseif strcmp(type ,  'inside')
    savename = sprintf('%si_%s_%d', savePath, 'sig_proc', i);
elseif strcmp(type , 'outside')
    savename = sprintf('%so_%s_%d', savePath, 'sig_proc', i);
end
save(savename, 'y_conv', 'y_fil', 'y_smooth');

% 
if draw
    subplot(2,2,3);
    plot(y_cut);
    subplot(2,2,4);
    plot(y_smooth);
end

end

