function [tau1, tau2, fi1dot_prev, fi2dot_prev, t_prev, ...
    P_hat_1_dot_prev,P_hat_2_dot_prev,P_hat_1_prev,P_hat_2_prev, e1,e2] ...
    = controller_model_based(l1,l2,lc1,lc2,m1,m2,ma,mb,I1,I2,g,t, ...
    fi1,fi1dot,fi2,fi2dot,fi1dot_prev, fi2dot_prev, t_prev,...
    P_hat_1_dot_prev,P_hat_2_dot_prev,P_hat_1_prev,P_hat_2_prev)



%-------------------------------------------------------------------------%
% Calculate Accelerations
%-------------------------------------------------------------------------%
dt = t-t_prev;
fi1dotdot = (fi1dot - fi1dot_prev)/dt;
fi2dotdot = (fi2dot - fi2dot_prev)/dt;
fi1dot_prev = fi1dot;
fi2dot_prev = fi2dot;


%-------------------------------------------------------------------------%
% Desired elliptical trajectory for the manipulator's end effector
%-------------------------------------------------------------------------%
alpha = 0.5;
beta = 0.23;
ycntr = 0.5;
zcntr = 0.5;
theta = deg2rad(45);
omega = 0.5;
y_1 = alpha * (cos(omega*t));
z_1 = beta * (sin(omega*t));
y_2 = cos(theta) * y_1 - sin(theta) * z_1;
z_2 = sin(theta) * y_1 + cos(theta) * z_1;
yd = ycntr + y_2;
zd = zcntr + z_2;


%-------------------------------------------------------------------------%
% Inverse Kinematics 
% Connecting manipulator angles to the tip's coords
%-------------------------------------------------------------------------%
c_invk = (zd^2 + yd^2 - l1^2 - l2^2)/(2*l1*l2);
s_invk = + sqrt(1 - c_invk^2); % (+) and (-) change elbow orientation
k1_invk = l2 + l1 * c_invk;
k2_invk = l1 * s_invk;
fi1d = atan2(zd, yd) - atan2(k2_invk, k1_invk);
fi2d = atan2(s_invk, c_invk);


%-------------------------------------------------------------------------%
% Tracking error
%-------------------------------------------------------------------------%
e1 = fi1d - fi1;
e2 = fi2d - fi2;
e1dot = 0 - fi1dot;
e2dot = 0 - fi2dot;


%-------------------------------------------------------------------------%
% Kp and Kv matrices
%-------------------------------------------------------------------------%
kp1 = 225;
kv1 = 30;
kp2 = 225;
kv2 = 30;


%-------------------------------------------------------------------------%
% Parameters to estimate - P hat matrix
%-------------------------------------------------------------------------%
% ma_h = 3; %1*ma;
% mb_h = 1.5; %1*mb;
ma_h = P_hat_1_prev; %1*ma;
mb_h = P_hat_2_prev; %1*mb;


%-------------------------------------------------------------------------%
% Ã matrix
%-------------------------------------------------------------------------%
g11 = 400;
g12 = 0;
g21 = 0;
g22 = 180;
G = [g11 g12 ; g21 g22];


%-------------------------------------------------------------------------%
% Wt matrix 
%-------------------------------------------------------------------------%
w11 = l1^2 * fi1dotdot + l1 * cos(fi1) * g;
w12 = (l1^2+ l2^2 +2*l1*l2*cos(fi2))*fi1dotdot + ...
    (l2^2 + l1*l2*cos(fi2))*fi2dotdot - ...
    l1*l2*sin(fi2)*(2*fi1dot+fi2dot)*fi2dot ...
    +(l1*cos(fi1)+l2*cos(fi1+fi2))*g;
w21 = 0;
w22 = (l2^2+l1*l2*cos(fi2))*fi1dotdot + l2^2*fi2dotdot + ...
    l1*l2*sin(fi2)*fi1dot^2 + l2*cos(fi1+fi2)*g;
W = [w11 w12; w21 w22];
Wt = W';


%-------------------------------------------------------------------------%
% M hat inverse matrix
%-------------------------------------------------------------------------%
a11_h = I1+I2+lc1.^2.*m1+l1.^2.*m2+lc2.^2.*m2+l1.^2.*ma_h+l1.^2.*mb_h+ ...
  l2.^2.*mb_h+2.*l1.*(lc2.*m2+l2.*mb_h).*cos(fi2);
a12_h = I2+lc2.^2.*m2+l2.^2.*mb_h+l1.*(lc2.*m2+l2.*mb_h).*cos(fi2);
a22_h = I2+lc2.^2.*m2+l2.^2.*mb_h;
M_hat = [a11_h a12_h; a12_h a22_h];
M_hat_inv = inv(M_hat);


%-------------------------------------------------------------------------%
% Psi matrix
%-------------------------------------------------------------------------%
psi1 = kv1/(1+kp1);
psi2 = kv2/(1+kp2);


%-------------------------------------------------------------------------%
% Filtered error ef
%-------------------------------------------------------------------------%
ef1 = e1dot + psi1 * e1;
ef2 = e2dot + psi2 * e2;
ef = [ef1 ef2]';


%-------------------------------------------------------------------------%
% Adaptation Law
%-------------------------------------------------------------------------%
P_hat_dot = G * Wt * M_hat_inv * ef;
% P_hat_dot = G * Wt / M_hat * ef;
% P_hat_dot = G * Wt * M_hat \ ef;
% Calculate the integrals
P_hat_1 = P_hat_1_prev + (P_hat_1_dot_prev + P_hat_dot(1)) * dt / 2;
P_hat_2 = P_hat_2_prev + (P_hat_2_dot_prev + P_hat_dot(2)) * dt / 2;

% Impose bounds
delta = 0.1;
p1_l = 2;
p2_l = 1;
p1_h = 6;
p2_h = 3;
if P_hat_1 < (p1_l - delta)
    P_hat_1 = p1_l;
elseif P_hat_1 > (p1_h + delta)
    P_hat_1 = p1_h;
end
if P_hat_2 < (p2_l - delta)
    P_hat_2 = p2_l;
elseif P_hat_2 > (p2_h + delta)
    P_hat_2 = p2_h;
end

ma_h = P_hat_1; 
mb_h = P_hat_2;

% Update the values for the next loop
P_hat_1_dot_prev = P_hat_dot(1);
P_hat_2_dot_prev = P_hat_dot(2);
P_hat_1_prev = P_hat_1;
P_hat_2_prev = P_hat_2;

% P_hat_1_dot_prev,P_hat_2_dot_prev,P_hat_1_prev,P_hat_2_prev,


%-------------------------------------------------------------------------%
% New M_hat
%-------------------------------------------------------------------------%
a11_h = I1+I2+lc1.^2.*m1+l1.^2.*m2+lc2.^2.*m2+l1.^2.*ma_h+l1.^2.*mb_h+ ...
  l2.^2.*mb_h+2.*l1.*(lc2.*m2+l2.*mb_h).*cos(fi2);
a12_h = I2+lc2.^2.*m2+l2.^2.*mb_h+l1.*(lc2.*m2+l2.*mb_h).*cos(fi2);
a22_h = I2+lc2.^2.*m2+l2.^2.*mb_h;


%-------------------------------------------------------------------------%
% New Q hat matrix
%-------------------------------------------------------------------------%
q1_h = -((-1).*g.*(lc1.*m1+l1.*(m2+ma_h+mb_h)).*cos(fi1)+(-1).*g.*( ...
  lc2.*m2+l2.*mb_h).*cos(fi1+fi2)+fi2dot.*(2.*fi1dot+fi2dot).* ...
  l1.*(lc2.*m2+l2.*mb_h).*sin(fi2));
q2_h = -((-1).*g.*(lc2.*m2+l2.*mb_h).*cos(fi1+fi2)+(-1).* ...
  fi1dot.^2.*l1.*(lc2.*m2+l2.*mb_h).*sin(fi2));


%-------------------------------------------------------------------------%
% Modified acceleration fi_e double dot
%-------------------------------------------------------------------------%
fid1_dot_dot = 0; % desired acceleration
fid2_dot_dot = 0; % desired acceleration
fie1_dot_dot = fid1_dot_dot + kv1 * e1dot + kp1 * e1;
fie2_dot_dot = fid2_dot_dot + kv2 * e2dot + kp2 * e2;


%-------------------------------------------------------------------------%
% Controller
%-------------------------------------------------------------------------%
tau1 = a11_h * fie1_dot_dot + a12_h * fie2_dot_dot + q1_h;
tau2 = a12_h * fie1_dot_dot + a22_h * fie2_dot_dot + q2_h;








