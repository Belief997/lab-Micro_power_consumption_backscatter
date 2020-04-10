% Ƶ�� 1 -> 100k 
f_tag = (10 ^ 5) / ratio;
% ������ 1M
fs = f_source * 10;
%�������� 10s
N = 10 * fs;
n = 0 : N-1;
%ʱ������
t = n / fs ;

y_tag = zeros(1, N);
for i=0:N-1
    % tag �� 1s �� 8s ���������
    if i >= (1 * fs) && i <= (9 * fs) 
        if mod(i, fs / f_tag) == 0
            y_tag(i) = round(unifrnd(0,1)) ;
        else
            y_tag(i) = y_tag(i - 1);
        end
    end
end
%���� tag �����ź�
% figure
% plot(t / ratio, y_tag);