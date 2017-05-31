function M = mass(t,x)

%-------------------------------------------------------------------------%
% All solvers solve systems of equations in the form 
% y' = f(t,y) or problems that involve a mass matrix, 
% M(t,y)y' = f(t,y). The ode23s solver can solve only 
% equations with constant mass matrices. ode15s and 
% ode23t can solve problems with a mass matrix that is 
% singular, i.e., differential-algebraic equations (DAEs).
%-------------------------------------------------------------------------%

%-------------------------------------------------------------------------%
% All solvers solve systems of equations in the form 
% y' = f(t,y) or problems that involve a mass matrix, 
% M(t,y)y' = f(t,y). The ode23s solver can solve only 
% equations with constant mass matrices. ode15s and 
% ode23t can solve problems with a mass matrix that is 
% singular, i.e., differential-algebraic equations (DAEs).
%-------------------------------------------------------------------------%

global ma mb m1 m2 I1 I2 l1 l2 lc1 lc2  

%-------------------------------------------------------------------------%
% x(1) = fi1 
% x(2) = fi1dot 
% x(3) = fi2
% x(4) = fi2dot
%-------------------------------------------------------------------------%

fi1 = x(1);
fi1dot = x(2);
fi2 = x(3);
fi2dot = x(4);


%-------------------------------------------------------------------------%
% Mass matrix
%-------------------------------------------------------------------------%
M = zeros(4,4);

%-------------------------------------------------------------------------%
% from Mathematica we get

M(2,2)=I1+I2+lc1.^2.*m1+l1.^2.*m2+lc2.^2.*m2+l1.^2.*ma+l1.^2.*mb+ ...
  l2.^2.*mb+2.*l1.*(lc2.*m2+l2.*mb).*cos(fi2);
M(2,4)=I2+lc2.^2.*m2+l2.^2.*mb+l1.*(lc2.*m2+l2.*mb).*cos(fi2);
M(4,4)=I2+lc2.^2.*m2+l2.^2.*mb;



%-------------------------------------------------------------------------%
% We add the symmetric elements
M(4,2)= M(2,4);


%-------------------------------------------------------------------------%
% We add aces for diagonal odd elements

M(1,1) = 1;
M(3,3) = 1;



