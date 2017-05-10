function [x_pred,P_pred]=predict_test(A,H,x_prev,P_prev,q)
% This function performs the prediction step 
% A is the state transition matrix
% H is measurmente matrix
% q is process noise (nx1, where n is number of states)
Q = diag(q);                % Process noise covariance matrix
x_pred = A*x_prev;          % Predict state
P_pred = A*P_prev*A'+Q;     % Predict error covariance matrix
end