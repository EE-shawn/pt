function [z_pred,x_pred,P_pred]=predict_test(A,H,x_prev,P_prev,q)
% A is the state transition matrix
% H is measurmente matrix
% q is process noise
Q = diag(q);
x_pred = A*x_prev;
z_pred = H*x_pred;
P_pred = A*P_prev*A'+Q;
end