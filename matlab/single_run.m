function [] = single_run(r_tagIn,r_tagOut, r_direct, draw_plot, isBeacon)
%SINGLE_RUN ��������˥������ʱ���㣬�Լ����λ���
%   
global c;
global ratio;
global f_source;
global P_source;
global y_source;
global y_sig;
global fs;
global dirname;

N = 10 * fs;
n = 0 : N-1;
t = n / fs ;

% r_tagIn = 6.5, r_tagOut = 3.25, r_direct = 9.086
%---------------------------------------------------
% ˥������

% [L_out] = ideal_decline(fx, r, G_tx, G_rx)
% fx(MHz), r(m), G_tx, G_rx
% tag ֮ǰ��tag ��������Դ��˥���ͽ��չ���
f_tx = f_source * ratio;
G_source = zeros(size(r_tagIn));
G_tag = zeros(size(r_tagIn));
L_tagIn = ideal_decline(f_tx * 10^(-6), r_tagIn, G_source, G_tag);
P_tagIn = P_source .* 10.^(-0.1 * L_tagIn);

% ֱ���Լ� tag ֮�󣬽�������ֱ��·���� tag ɢ���˥���ͽ��չ���
% r_tagOut = 3.25;
% r_direct = 9.086;
G_rx = 0;
L_tagOut = ideal_decline(f_tx * 10^(-6), r_tagOut, G_tag, G_rx);
L_direct = ideal_decline(f_tx * 10^(-6), r_direct, G_source, G_rx);
P_tag = P_tagIn .* 10.^(-0.1 * L_tagOut);
P_direct = P_source .* 10.^(-0.1 * L_direct);

% ��������
if isBeacon
    filename = sprintf('%sBeacon_%s', dirname, 'decline');
else 
    filename = sprintf('%s%s', dirname, 'decline');
end
save(filename, 'L_tagIn', 'L_tagOut', 'L_direct');

% ʹ�� tag ǰ��ľ���ͼ���˥����������
% L_test = ideal_decline(f_tx * 10^(-6), r_tagIn + r_tagOut, G_source, G_rx);
% P_test = P_source * 10^(-0.1 * L_test); 
%--------------------------------------
% �ֱ𻭳�����·��˥������ź�
decline_direct = y_source .* repmat(reshape((P_direct ./ P_source)', [], 1), 1, length(y_source));
decline_tag = y_sig .* repmat(reshape((P_tag ./ P_source)', [], 1), 1, length(y_sig));

if draw_plot
    % decline_tag = y_sig * (P_test / P_source);
    figure('NumberTitle', 'off', 'Name', '����·��˥��');
    subplot(2, 1, 1);
    plot(t / ratio, decline_direct);
    subplot(2, 1, 2);
    plot(t / ratio, decline_tag);
end


%-------------------------------------
% ������ʱ

delay_direct = r_direct ./ c ;
delay_reflect = (r_tagIn + r_tagOut) ./ c;
% ʱ�������ȥֱ�䣨�Ǹ���
delta_t = delay_reflect - delay_direct;
disp('delta t is');
disp(delta_t);

% ��������
if isBeacon
    filename = sprintf('%sBeacon_%s', dirname, 'delay');
else
    filename = sprintf('%s%s', dirname, 'delay');  
end
save(filename, 'delay_reflect', 'delay_direct');

% ����������ʱ����Ӻ�

 for i=1:length(delay_reflect(:)) 
    shift_reflect = zeros(1, round(delay_reflect(i) * ratio * fs));
    shift_direct = zeros(1, round(delay_direct(i) * ratio * fs));
    % �Է���ʱ��Ϊ��׼����չʱ����
    n_plot = 0 : (length(shift_reflect) + N - 1);
    t_plot = n_plot / fs;
    sig_dir = [shift_direct decline_direct(i,:) zeros(1, length(shift_reflect) - length(shift_direct))];
    sig_tag = [shift_reflect decline_tag(i,:)];
    sig_rx = [shift_direct decline_direct(i,:) zeros(1, length(shift_reflect) - length(shift_direct))] + [shift_reflect decline_tag(i,:)];
    % ��������
    if isBeacon
        filename = sprintf('%sBeacon_%s_%s', dirname, 'sig_rx', num2str(i));
    else
        filename = sprintf('%s%s_%s', dirname, 'sig_rx', num2str(i));
    end
    save(filename, 'sig_dir', 'sig_tag', 'sig_rx');
        
    if draw_plot && false
        % ������ʱ��Ĳ���
        figure('NumberTitle', 'off', 'Name', '������ʱ����Ӻ�');
        
        subplot(3, 1, 1);
        % plot(t / ratio, decline_direct);
        plot(t_plot / ratio, sig_dir);
        title('ֱ��·��');
        subplot(3, 1, 2);
        % plot(t / ratio, decline_tag);
        plot(t_plot / ratio, sig_tag);
        title('Tag ·��');
        subplot(3, 1, 3);
        % plot(t / ratio, decline_tag);
        plot(t_plot / ratio, sig_rx);
        title('����');
    end
end

%--------------------------------------





end
