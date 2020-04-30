function [d1, d2, d3] = xy2d(x, y, reciver, location)
%XY2D 通过 x y 的值计算对应的 d 
% d1-front, d2-behind, d3-direct
global H0;
global H1;
global H2;
global H3;
global Length;
global Hight;
global D3;

h1 = H1 * ones(size(x));
h2 = H2 * ones(size(x));
h3 = H3 * ones(size(x));

% tagIn
if strcmp(location, 'inside') || strcmp(location, 'outside')
    d1 = sqrt(x .^2 + y .^2 + h1 .^2);
elseif strcmp(location, 'up')
    d1 = sqrt(x .^2 + y .^2 + (Hight - H0 - h1) .^2);
elseif strcmp(location, 'down')
    d1 = sqrt(x .^2 + y .^2 + (Hight + h1) .^2);
else
       disp('location invalid');
       quit;
end

% tagOut
if reciver == 1
    if strcmp(location, 'inside')
        d2 = sqrt(x .^2 + (Length * ones(size(x)) - y) .^2 + h2 .^2);
    elseif strcmp(location, 'outside')
        d2 = sqrt(x .^2 + (-1 * Length * ones(size(x)) - y) .^2 + h2 .^2);
    elseif strcmp(location, 'up')
        d2 = sqrt(x .^2 + (Length * ones(size(x)) - y) .^2 + (Hight - H0 - h2) .^2);
    elseif strcmp(location, 'down')
        d2 = sqrt(x .^2 + (Length * ones(size(x)) - y) .^2 + (Hight + h2) .^2);
    else
       disp('location invalid');
       quit;
    end
elseif reciver == 2
    if strcmp(location, 'inside')
        d2 = sqrt(x .^2 + (Length * ones(size(x)) - y) .^2 + h3 .^2);
    elseif strcmp(location, 'outside')
        d2 = sqrt(x .^2 + (-1 * Length * ones(size(x)) - y) .^2 + h3 .^2);
    elseif strcmp(location, 'up')
        d2 = sqrt(x .^2 + (Length * ones(size(x)) - y) .^2 + (Hight - H0 - h3) .^2);
    elseif strcmp(location, 'down')
        d2 = sqrt(x .^2 + (Length * ones(size(x)) - y) .^2 + (Hight + h3) .^2);
    else
        disp('location invalid');
        quit;
    end
else
    disp('err: reciver number is invalid !');
    quit;
end

% direct
d3 = D3 * ones(size(x));

end

