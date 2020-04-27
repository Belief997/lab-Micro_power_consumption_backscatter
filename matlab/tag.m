% Ƶ�� 1 -> 100k 
f_tag = (10 ^ 5) / ratio;
% ������ 1M
fs = f_source * 10;
%�������� 10s
N = 10 * fs;
n = 0 : N-1;
%ʱ������
t = n / fs ;
sig_bin = [1 0 1 0 1 1 0 1 1];
index = 1;
y_tag = zeros(1, N);
for i=0:N-1
    % tag �� 1s �� 8s ���������
    if i >= (1 * fs) && i <= (9 * fs) 
        if mod(i, fs / f_tag) == 0
            y_tag(i) = sig_bin(index) ;
            index = index + 1;
        else
            y_tag(i) = y_tag(i - 1);
        end
    end
end
%���� tag �����ź�
% figure
% plot(t / ratio, y_tag);

% �����ź�
global dirname;
filename = sprintf('%s%s', dirname, 'y_tag');

global run_type;
if run_type == 'bt'
    save(filename, 'y_tag');
end
