% Simulation and animation of a two-segmented manipulator
% Dynamics derived with Mathematica 
% Mathematica file:"Two_segmented_manipulator_dynamics.nb"
% Created by Konstantinos Machairas (kmach@central.ntua.gr) - 5/2017


clc
clear all
global ma mb m1 m2 I1 I2 l1 l2 lc1 lc2 tfinal g
global t_prev fi1dot_prev fi2dot_prev
global P_hat_1_dot_prev P_hat_2_dot_prev P_hat_1_prev P_hat_2_prev


%-------------------------------------------------------------------------%
% Simulation duration
%-------------------------------------------------------------------------%
tstart = 0;
tfinal = 10;


%-------------------------------------------------------------------------%
% Define physical parameters of the system
%-------------------------------------------------------------------------%
ma = 4;
mb = 2;
m1 = 0;
m2 = 0;

I1 = 0;
I2 = 0;

l1 = 1;
l2 = 1;

lc1 = l1/2;
lc2 = l2/2;

% Gravity
g = 9.81;


%-------------------------------------------------------------------------%
% Initialize vectors
%-------------------------------------------------------------------------%
t_prev = -0.00001;
fi1dot_prev = 0;
fi2dot_prev = 0;
P_hat_1_dot_prev = 0;
P_hat_2_dot_prev = 0;
P_hat_1_prev = 3;
P_hat_2_prev = 1.5;



%-------------------------------------------------------------------------%
% Solve the Equations of Motion  
% Matrix form: M(x,x') * x' = F(x,x')
%-------------------------------------------------------------------------%
% x(1) = fi1 
% x(2) = fi1dot 
% x(3) = fi2
% x(4) = fi2dot
%-------------------------------------------------------------------------%

% Set initial conditions
fi1_0 = deg2rad(-05);
fi2_0 = deg2rad(100);
fi1dot_0 = 0;
fi2dot_0 = 0;

% Vector of initial conditions.
x0 = [fi1_0, fi1dot_0, fi2_0, fi2dot_0];

% Options for ODE
options = odeset('MaxStep',1e-3,'AbsTol',1e-6,'RelTol',1e-6,'Mass',@mass);

% Set the time interval
tspan = [tstart tfinal];

% Solve ODE
[t,x] = ode45(@dynamics,tspan,x0,options);

% Save the results
xout = x; 
tout = t;


%-------------------------------------------------------------------------%
% Post Processing
%-------------------------------------------------------------------------%
post_processing


%-------------------------------------------------------------------------%
% Animation
%-------------------------------------------------------------------------%
animation


%-------------------------------------------------------------------------%
% Plots
%-------------------------------------------------------------------------%
plots
