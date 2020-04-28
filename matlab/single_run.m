function [] = single_run(r_tagIn,r_tagOut, r_direct, draw_plot, type)
%SINGLE_RUN 对任意点的衰减和延时计算，以及波形绘制
%   
global c;
global ratio;
global f_source;
% global P_source;
global y_source;
global y_sig;
global fs;
global dirname;

N = 10 * fs;
n = 0 : N-1;
t = n / fs ;

%---------------------------------------------------
% 衰减计算
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

L_tag = L_tagIn + L_tagOut;
disp('L_tag is');
disp(L_tag);

% 保存数据
if strcmp(type , 'beacon')
    filename = sprintf('%sBeacon_%s', dirname, 'decline');
elseif strcmp(type ,  'tag')
    filename = sprintf('%s%s', dirname, 'decline');
elseif strcmp(type ,  'inside')
    filename = sprintf('.\\DataSet\\i_%s', 'decline');
elseif strcmp(type ,  'outside')
    filename = sprintf('.\\DataSet\\o_%s', 'decline');
end
save(filename, 'L_tagIn', 'L_tagOut', 'L_direct');

%--------------------------------------
% 分别画出每个点两条路径衰减后的信号，得到的每一行代表一个点
decline_direct = y_source .* repmat(reshape((10.^(-0.1 * L_direct))', [], 1), 1, length(y_source));
decline_tag = y_sig .* repmat(reshape((10.^(-0.1 * L_tag))', [], 1), 1, length(y_sig));

if draw_plot
    % decline_tag = y_sig * (P_test / P_source);
    figure('NumberTitle', 'off', 'Name', '两条路径衰减');
    subplot(2, 1, 1);
    plot(t / ratio, decline_direct);
    subplot(2, 1, 2);
    plot(t / ratio, decline_tag);
end


%-------------------------------------
% 计算延时

delay_direct = r_direct ./ c ;
delay_reflect = (r_tagIn + r_tagOut) ./ c;
% 时间差，反射减去直射（非负）
delta_t = delay_reflect - delay_direct;
disp('delta t is');
disp(delta_t);

% 保存数据
if strcmp(type , 'beacon')
    filename = sprintf('%sBeacon_%s', dirname, 'delay');
elseif strcmp(type , 'tag')
    filename = sprintf('%s%s', dirname, 'delay');  
elseif strcmp(type ,  'inside')
    filename = sprintf('.\\DataSet\\i_%s', 'delay');
elseif strcmp(type ,  'outside')
    filename = sprintf('.\\DataSet\\o_%s', 'delay');
end
save(filename, 'delay_reflect', 'delay_direct');

% 画出两个延时及其加和

 for i=1:length(delay_reflect(:)) 
    shift_reflect = zeros(1, round(delay_reflect(i) * ratio * fs));
    shift_direct = zeros(1, round(delay_direct(i) * ratio * fs));
    % 以反射时长为基准，扩展时间轴
    n_plot = 0 : (length(shift_reflect) + N - 1);
    t_plot = n_plot / fs;
    sig_dir = [shift_direct decline_direct(i,:) zeros(1, length(shift_reflect) - length(shift_direct))];
    sig_tag = [shift_reflect decline_tag(i,:)];
    sig_rx = sig_dir + sig_tag;
    % 保存数据
    if strcmp(type ,  'beacon')
        filename = sprintf('%sBeacon_%s_%d', dirname, 'sig_rx', i);
    elseif strcmp(type ,  'tag')
        filename = sprintf('%s%s_%d', dirname, 'sig_rx', i);
    elseif strcmp(type , 'inside')
        filename = sprintf('.\\DataSet\\i_%s_%d', 'sig_rx', i);
    elseif strcmp(type ,  'outside')
        filename = sprintf('.\\DataSet\\o_%s_%d', 'sig_rx', i);
    end
    save(filename, 'sig_dir', 'sig_tag', 'sig_rx');
        
    if draw_plot || true
        % 画出延时后的波形
        figure('NumberTitle', 'off', 'Name', '两个延时及其加和');
        
        subplot(3, 1, 1);
        % plot(t / ratio, decline_direct);
        plot(t_plot / ratio, angle(sig_dir));
        title('直射路径');
        subplot(3, 1, 2);
        % plot(t / ratio, decline_tag);
        plot(t_plot / ratio, angle(sig_tag));
        title('Tag 路径');
        subplot(3, 1, 3);
        % plot(t / ratio, decline_tag);
        plot(t_plot / ratio, angle(sig_rx));
        title('叠加');
    end
end


end

