function [z_corr,x_corr,P_corr] = correct_test(H,x_prev,P_prev,z,r)
% A is the state transitino matrix
% H is measurement matrix
% r is the process noise
% x_prev is the previous state estimate
% z is current measurement
R = diag(r);

resd = z-H*x_prev;
S = H*P_prev*H'+R;
K = P_prev*H'/S;
x_corr = x_prev+K*resd;
z_corr = H*x_corr;
P_corr = P_prev-K*H*P_prev;
end