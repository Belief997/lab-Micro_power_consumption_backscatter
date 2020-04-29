function [] = single_run(r_tagIn, r_tagOut, r_direct, draw_plot, type, N_rx)
%SINGLE_RUN 对任意点的衰减和延时计算，以及波形绘制
%   
global c;
global ratio;
global f_source;
global P_source;
global P_noise;
global y_source;
global y_tag;
% global y_sig;
global fs;
global dirname;

N = 10 * fs;
n = 0 : N-1;
t = n / fs ;

%% 衰减计算
% 
% [L_out] = ideal_decline(fx, r, G_tx, G_rx)
% fx(MHz), r(m), G_tx, G_rx
% tag 之前，tag 接收来自源的衰减和接收功率
f_tx = f_source * ratio;
G_source = zeros(size(r_tagIn));
G_tag = zeros(size(r_tagIn));
L_tagIn = ideal_decline(f_tx * 10^(-6), r_tagIn, G_source, G_tag);
% P_tagIn = P_source .* 10.^(-0.1 * L_tagIn);

% 直射以及 tag 之后，接收来自直射路径和 tag 散射的衰减和接收功率
G_rx = 0;
L_tagOut = ideal_decline(f_tx * 10^(-6), r_tagOut, G_tag, G_rx);
L_direct = ideal_decline(f_tx * 10^(-6), r_direct, G_source, G_rx);
% P_tag = P_tagIn .* 10.^(-0.1 * L_tagOut);
% P_direct = P_source .* 10.^(-0.1 * L_direct);

% L_all = [L_tagIn L_tagOut L_direct];
L_tag = L_tagIn + L_tagOut;
disp('L_tag is');
disp(L_tag);

% if strcmp(type, 'tag')
%     [X,Y] = meshgrid(0:3, 2:4);
%     
% end

% 保存数据
if strcmp(type , 'beacon')
    filename = sprintf('%sBeacon_%s_%d', dirname, 'decline', N_rx);
elseif strcmp(type ,  'tag')
    filename = sprintf('%s%s_%d', dirname, 'decline', N_rx);
elseif strcmp(type ,  'inside')
    filename = sprintf('.\\DataSet\\i_%s_%d', 'decline', N_rx);
elseif strcmp(type ,  'outside')
    filename = sprintf('.\\DataSet\\o_%s_%d', 'decline', N_rx);
end
save(filename, 'L_tagIn', 'L_tagOut', 'L_direct');

%--------------------------------------
% 分别画出每个点两条路径衰减后的信号，得到的每一行代表一个点
% decline_direct = y_source .* repmat(reshape((10.^(-0.1 * L_direct))', [], 1), 1, length(y_tag));
% decline_tagIn = y_source .* repmat(reshape((10.^(-0.1 * L_tagIn))', [], 1), 1, length(y_tag));
% noise = wgn(numel(L_tagIn), length(y_source), P_noise, 'complex');

% if draw_plot
%     % decline_tag = y_sig * (P_test / P_source);
%     figure('NumberTitle', 'off', 'Name', '两条路径衰减');
%     subplot(2, 1, 1);
%     plot(t / ratio, decline_direct);
%     subplot(2, 1, 2);
%     plot(t / ratio, decline_tagIn);
% end


%% 计算延时
% 
% delay_all = d ./ c;
delay_direct = r_direct ./ c;
delay_reflect = (r_tagIn + r_tagOut) ./ c;
delay_tagIn = r_tagIn ./ c;
% 时间差，反射减去直射（非负）
delta_t = delay_reflect - delay_direct;

disp('delta t is');
disp(delta_t);

% 保存数据
if strcmp(type , 'beacon')
    filename = sprintf('%sBeacon_%s_%d', dirname, 'delay', N_rx);
elseif strcmp(type , 'tag')
    filename = sprintf('%s%s_%d', dirname, 'delay', N_rx);  
elseif strcmp(type ,  'inside')
    filename = sprintf('.\\DataSet\\i_%s_%d', 'delay', N_rx);
elseif strcmp(type ,  'outside')
    filename = sprintf('.\\DataSet\\o_%s_%d', 'delay', N_rx);
end
save(filename, 'delay_tagIn', 'delay_reflect', 'delay_direct', 'delta_t');

% 延时 计数
N_delayIn =  round(delay_tagIn * ratio * fs);
N_delayDir = round(delta_t * ratio * fs);

% 画出两个延时及其加和
 for i=1:length(delay_reflect(:)) 
     % 时延 截取
    sig_direct = y_source(1, N_delayDir + 1 : N_delayDir + length(y_tag));
    sig_reflect = y_source(1, N_delayIn + 1 : N_delayIn + length(y_tag));

    decline_direct = sig_direct * 10^(-0.1 * L_direct(i));
    decline_tagIn = sig_reflect * 10^(-0.1 * L_tagIn(i));
    
    sig_tagOut = decline_tagIn .* y_tag;
    decline_tagOut = sig_tagOut * 10^(-0.1 * L_tagOut(i));
    
    sig_rx = decline_direct + decline_tagOut;
    
    % 以反射时长为基准，扩展时间轴
    n_plot = 0 : length(y_tag)-1;
    t_plot = n_plot / fs;
    % 保存数据
    if strcmp(type ,  'beacon')
        filename = sprintf('%sBeacon_%s_%d_%d', dirname, 'sig_rx', N_rx, i);
    elseif strcmp(type ,  'tag')
        filename = sprintf('%s%s_%d_%d', dirname, 'sig_rx', N_rx, i);
    elseif strcmp(type , 'inside')
        filename = sprintf('.\\DataSet\\i_%s_%d_%d', 'sig_rx', N_rx, i);
    elseif strcmp(type ,  'outside')
        filename = sprintf('.\\DataSet\\o_%s_%d_%d', 'sig_rx', N_rx, i);
    end
    save(filename, 'decline_direct', 'decline_tagOut', 'sig_rx');
        
    if draw_plot && false
        % 画出延时后的波形
        figure('NumberTitle', 'off', 'Name', '两个延时及其加和');
        
        subplot(3, 1, 1);
        % plot(t / ratio, decline_direct);
        plot(t_plot / ratio, angle(decline_direct));
        title('直射路径');
        subplot(3, 1, 2);
        % plot(t / ratio, decline_tag);
        plot(t_plot / ratio, angle(decline_tagOut));
        title('Tag 路径');
        subplot(3, 1, 3);
        % plot(t / ratio, decline_tag);
        plot(t_plot / ratio, angle(sig_rx));
        title('叠加');
    end
end


end

