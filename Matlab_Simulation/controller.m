function [tau1, tau2] = controller(l1,l2,m1,m2,mb,I1,I2,g,t, ...
    fi1,fi1dot,fi2,fi2dot)


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
% Controller
%-------------------------------------------------------------------------%
kp = 1000;
kd = 100;

% fi1d = deg2rad(10);
% fi2d = deg2rad(20);

tau1 = kp*(fi1d - fi1) - kd*(fi1dot);
tau2 = kp*(fi2d - fi2) - kd*(fi2dot);







