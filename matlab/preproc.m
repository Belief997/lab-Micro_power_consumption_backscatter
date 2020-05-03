function [] = preproc(filename, i, draw, type)
%PREPROC 对接收的信号做预处理，求一个幅度，求相位差
%   

global dirname;
savePath = dirname;
% xxx -> xxx_i -> .\xxx\xxx_i
if strcmp(type ,  'inside') || strcmp(type ,  'outside') || strcmp(type ,  'up') || strcmp(type ,  'down')
    savePath = '.\DataSet\';
end
file_name = sprintf('%s%s_1_%d.mat',savePath, filename, i);
if strcmp(type ,  'tag')
    Sig_rx_1 = [cell2mat(struct2cell(load(file_name, 'sig_rx_1'))); ...
                cell2mat(struct2cell(load(file_name, 'sig_rx_2'))); ...
                cell2mat(struct2cell(load(file_name, 'sig_rx_3')));];% ...
%                 cell2mat(struct2cell(load(file_name, 'sig_rx_4')))];
else
    Sig_rx_1 = cell2mat(struct2cell(load(file_name, 'sig_rx')));
end
file_name = sprintf('%s%s_2_%d.mat',savePath, filename, i);
if strcmp(type ,  'tag')
    Sig_rx_2 = [cell2mat(struct2cell(load(file_name, 'sig_rx_1'))); ...
                cell2mat(struct2cell(load(file_name, 'sig_rx_2'))) ; ...
                cell2mat(struct2cell(load(file_name, 'sig_rx_3')))]; %...
%                 cell2mat(struct2cell(load(file_name, 'sig_rx_4')))];
else
    Sig_rx_2 = cell2mat(struct2cell(load(file_name, 'sig_rx')));
end
%% 求幅度
    Amp_1 = abs(Sig_rx_1);
    Amp_2 = abs(Sig_rx_2);
    
if draw
    figure;
    subplot(2, 1, 1);
    plot(Amp_1);
    subplot(2, 1, 2);
    plot(Amp_2);
end
%% 求相位差
An_1 = angle(Sig_rx_1);
An_2 = angle(Sig_rx_2);
An_diff = An_1 - An_2;

if draw 
    figure
    subplot(3, 1, 1);
    plot(An_1);
    subplot(3, 1, 2);
    plot(An_2);
    subplot(3, 1, 3);
    plot(An_diff);
end

% 保存数据

if strcmp(type ,  'beacon')
    savename = sprintf('%sBeacon_%s_%d', savePath, 'sig_proc', i);
elseif strcmp(type , 'tag')
    savename = sprintf('%s%s_%d', savePath, 'sig_proc', i);  
elseif strcmp(type ,  'inside')
    savename = sprintf('%si_%s_%d', savePath, 'sig_proc', i);
elseif strcmp(type , 'outside')
    savename = sprintf('%so_%s_%d', savePath, 'sig_proc', i);
elseif strcmp(type , 'up')
    savename = sprintf('%su_%s_%d', savePath, 'sig_proc', i); 
elseif strcmp(type , 'down')
    savename = sprintf('%sd_%s_%d', savePath, 'sig_proc', i);     
end
save(savename, 'Amp_1', 'Amp_2', 'An_diff');

end

