function [L_out] = ideal_decline(fx, r, G_tx, G_rx)
%IDEAL_DECLINE 自由空间电磁波的理想功率衰减公式 
%   L(dB) = 10 lg(P_tx / P_rx)
%         = 32.45 + 20 lg(f)(MHz) + 20 lg(r)(km) \
%           - G_tx(dB) - G_rx(dB)
%         = 32.45 + 20 lg(f)(MHz) + 20 lg(r(m)*10^-3) \
%           - G_tx(dB) - G_rx(dB)
L_out = 32.45 + 20 * log10(fx) + 20 * log10(r .* 10^(-3)) - G_tx - G_rx;

end

