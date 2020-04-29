function [d1, d2, d3] = xy2d(x, y, reciver)
%XY2D 通过 x y 的值计算对应的 d 
% d1-front, d2-behind, d3-direct
global H1;
global H2;
global H3;
global Length;
global D3;

h1 = H1 * ones(size(x));
h2 = H2 * ones(size(x));
h3 = H3 * ones(size(x));

d1 = sqrt(x .^2 + y .^2 + h1 .^2);
if reciver == 1
    d2 = sqrt(x .^2 + (Length * ones(size(x)) - y) .^2 + h2 .^2);
elseif reciver == 2
    d2 = sqrt(x .^2 + (Length * ones(size(x)) - y) .^2 + h3 .^2);
else
    disp('err: reciver number is invalid !');
    quit;
end
d3 = D3 * ones(size(x));

end

