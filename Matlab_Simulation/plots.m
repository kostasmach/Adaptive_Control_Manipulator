%-------------------------------------------------------------------------%
% Plots
%-------------------------------------------------------------------------%

%%
f2 = figure(2);
set(f2, 'Position', [100, 70, 1000, 700]);
clf(f2);
% Background color
set(gcf,'color','w');


% Plot tracking errors
subplot(2,1,1)
p1 = plot(tout, error_fi1,'LineWidth',1);
hold on
p2 = plot(tout, error_fi2,'LineWidth',1);
ylabel('e') 
xlabel('t (s)') 
title('Tracking errors - Time')
grid on
legend([p1 p2], 'e1','e2');


% Plot parameter estimation 
subplot(2,1,2)
p1 = plot(tout, ma_hat_param,'LineWidth',1);
hold on
p2 = plot(tout, mb_hat_param,'LineWidth',1);
p3 = plot([0,15], [4 4],'LineWidth',1);
p4 = plot([0,15], [2 2],'LineWidth',1);
ylabel('ma, mb') 
xlabel('t (s)') 
title('Unknown Parameters - Time')
grid on
legend([p1 p3 p2 p4], 'ma hat','ma','mb hat','mb');



%%


f3 = figure(3);
set(f3, 'Position', [100, 70, 1000, 700]);
clf(f3);
% Background color
set(gcf,'color','w');

% Plot tracking errors
p1 = plot(tout, Torques(:,1),'LineWidth',1);
hold on
p2 = plot(tout, Torques(:,2),'LineWidth',1);
ylabel('tau1, tau2 (Nm)') 
xlabel('t (s)') 
title('Control Torques - Time')
grid on
legend([p1 p2], 'tau1','tau2');

