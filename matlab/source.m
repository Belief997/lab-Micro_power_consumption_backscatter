% ������ 10 ��
global fs;
A = 1;
E = 0 * A;
%�������� 10s
Ns = 10; % �������ڲ�������
fs = f_source * Ns;
N = round(10 * fs);
n = 0 : N-1;
%ʱ������
% global t;
t = n / fs ;
phase = linspace(0, 2 * pi, Ns + 1); % ������λ

global y_source;
y_source_i = cos(phase(mod(n, Ns)+1)) + E;
y_source_q = sin(phase(mod(n, Ns)+1)) + E;
y_source = A * (y_source_i + y_source_q * 1i);
%�����ź�Դ
% figure
% plot(t / ratio, y_source);

% �����ź�
global dirname;
filename = sprintf('%s%s', dirname, 'y_source');

global run_type;
if run_type == 'bt'
    save(filename, 'y_source');
end



