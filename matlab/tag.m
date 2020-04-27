% 频率 1 -> 100k 
f_tag = (10 ^ 5) / ratio;
% 采样率 1M
fs = f_source * 10;
%采样点数 10s
N = 10 * fs;
n = 0 : N-1;
%时间序列
t = n / fs ;
sig_bin = [1 0 1 0 1 1 0 1 1];
index = 1;
y_tag = zeros(1, N);
for i=0:N-1
    % tag 在 1s 到 8s 内随机调制
    if i >= (1 * fs) && i <= (9 * fs) 
        if mod(i, fs / f_tag) == 0
            y_tag(i) = sig_bin(index) ;
            index = index + 1;
        else
            y_tag(i) = y_tag(i - 1);
        end
    end
end
%画出 tag 编码信号
% figure
% plot(t / ratio, y_tag);

% 保存信号
global dirname;
filename = sprintf('%s%s', dirname, 'y_tag');

global run_type;
if run_type == 'bt'
    save(filename, 'y_tag');
end
