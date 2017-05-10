function [x_corr,P_corr] = correct_test(H,x_prev,P_prev,z,r)
% This function performs the correction step involved in Kalman Filtering
% A is the state transition matrix
% H is measurement matrix
% r is the process noise (mx1 vector m denotes dimension of measurement)
% x_prev is the previous state estimate
% z is current measurement
% P_prev is the previous error covariance matrix

R = diag(r);                % Measurement Covariance 

resd = z-H*x_prev;          % residual
S = H*P_prev*H'+R;          % innovation covariance
K = P_prev*H'/S;            % Kalman Gain
x_corr = x_prev+K*resd;     % Correction in state estimate
P_corr = P_prev-K*H*P_prev; % Correction in the error covariance 
end