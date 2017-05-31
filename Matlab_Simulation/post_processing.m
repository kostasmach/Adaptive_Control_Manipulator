%-------------------------------------------------------------------------%
% Post Processing
%-------------------------------------------------------------------------%

global tfinal
global t_prev fi1dot_prev fi2dot_prev
global P_hat_1_dot_prev P_hat_2_dot_prev P_hat_1_prev P_hat_2_prev

n = size(tout,1); % data size

%-------------------------------------------------------------------------%
% Initialization of new vectors
%-------------------------------------------------------------------------%
Torques = [];
tout2 = [];
xout2 = [];
ma_hat_param = [];
mb_hat_param = [];
error_fi1 = [];
error_fi2 = [];


for i_p = 1 : n
  
%-------------------------------------------------------------------------%
% Read the output vector element by element
%-------------------------------------------------------------------------%
% x(1) = fi1 
% x(2) = fi1dot 
% x(3) = fi2
% x(4) = fi2dot
%-------------------------------------------------------------------------%
fi1 = x(i_p,1);
fi1dot = x(i_p,2); 
fi2 = x(i_p,3);
fi2dot = x(i_p,4);


%-------------------------------------------------------------------------%
% Read the time vector element by element
%-------------------------------------------------------------------------%  
t_i = tout(i_p);


%-------------------------------------------------------------------------%
% Control Torques - Recalculating torques by calling the function
%-------------------------------------------------------------------------%

[tau1, tau2, fi1dot_prev, fi2dot_prev,t_prev,P_hat_1_dot_prev,...
    P_hat_2_dot_prev,P_hat_1_prev,P_hat_2_prev,e1,e2] = ...
    controller_model_based(l1,l2,lc1,lc2,m1,m2,ma,mb,...
    I1,I2,g,t_i,fi1,fi1dot,fi2,fi2dot,...
    fi1dot_prev,fi2dot_prev,t_prev,P_hat_1_dot_prev,...
    P_hat_2_dot_prev,P_hat_1_prev,P_hat_2_prev);


%-------------------------------------------------------------------------%
% Save torques to vectors
%-------------------------------------------------------------------------%
tau_i = [tau1, tau2];
Torques = [Torques; tau_i];
xout2 = [xout2; [fi1,fi1dot,fi2, fi2dot]];
tout2 = [tout2; t_i];
ma_hat_param = [ma_hat_param;P_hat_1_prev];
mb_hat_param = [mb_hat_param;P_hat_2_prev];
error_fi1 = [error_fi1; e1];
error_fi2 = [error_fi2; e2];


%-------------------------------------------------------------------------%
% Printing
%-------------------------------------------------------------------------%
if ~mod(i_p,200)
    clc
    message2 = ['Post processing time: ',num2str(t_i),'s /',num2str(tfinal),'s'];
    disp(message2)
end

end