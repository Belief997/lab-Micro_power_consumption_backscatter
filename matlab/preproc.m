function [] = preproc(filename, i, draw, isBeacon)
%PREPROC 对接收的信号做预处理，去载波，滤波
%   

global dirname;
% xxx -> xxx_i -> .\xxx\xxx_i
file_name = sprintf('%s_%s', filename, num2str(i));
file_name = strcat(dirname, file_name);
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
[b,a]=sos2tf(SOS,G);
y_fil=filter(b,a, y_conv);

% 截取
y_cut = y_fil(1,1000:2400000);
y_smooth = smooth(y_cut, 800);

% 保存数据
if isBeacon
    savename = sprintf('%sBeacon_%s_%s', dirname, 'sig_proc', num2str(i));
else
    savename = sprintf('%s%s_%s', dirname, 'sig_proc', num2str(i));  
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

