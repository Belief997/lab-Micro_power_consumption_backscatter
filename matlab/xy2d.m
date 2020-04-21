function [d1, d2, d3] = xy2d(x,y)
%XY2D ͨ�� x y ��ֵ�����Ӧ�� d 
%   �˴���ʾ��ϸ˵��
global H1;
global H2;
global Length;
global D3;

h1 = H1 * ones(size(x));
h2 = H2 * ones(size(x));
d1 = sqrt(x .^2 + y .^2 + h1 .^2);
d2 = sqrt(x .^2 + (Length * ones(size(x)) - y) .^2 + h2 .^2);
d3 = D3 * ones(size(x));

end

