% Animation

f1 = figure(1);
set(f1, 'Position', [100, 70, 1000, 700]);
clf(f1);

% Background color
set(gcf,'color','w');

n = size(xout,1);

fi1 = xout(:,1);
fi2 = xout(:,3);


% Segment 1 CoM coordinates
y1 = lc1.*cos(fi1);
z1 = lc1.*sin(fi1);
% Mass a coordinates
ya = l1.*cos(fi1);
za = l1.*sin(fi1);
% Segment 2 CoM coordinates
y2 = ya + lc2.*cos(fi1+fi2);
z2 = za + lc2.*sin(fi1+fi2);
% Mass b coordinates
yb = ya + l2.*cos(fi1+fi2);
zb = za + l2.*sin(fi1+fi2);


% Animation Step
step = 500;


for i = 1:step:n-1

    % Plot ground
    dg1 = 0.2;
    dg2 = -0.07;
    v1 = [-dg1 , dg2];  % up left corner
    v2 = [dg1 , dg2];   % up right corner
    v3 = [dg1 , 0];  % down right corner
    v4 = [-dg1 , 0]; % down left corner
    abcissas = [v1(1) v2(1) v3(1) v4(1)]; 
    ordinates = [v1(2) v2(2) v3(2) v4(2)];
    fill(abcissas, ordinates, [0.9 0.9 0.9],'EdgeColor','none');
    hold on;
    plot([-0.2 0.2],[0 0],'k')

    % Plot mass a
    plot(ya(i),za(i),'ko','MarkerFaceColor','w','MarkerSize',20)
    
    % Plot mass b
    plot(yb(i),zb(i),'ko','MarkerFaceColor','w','MarkerSize',10)
    
    % Plot segments
    plot([0 ya(i)],[0 za(i)],'k','LineWidth',2)
    plot([ya(i) yb(i)],[za(i) zb(i)],'k','LineWidth',2)
    
    % Plot joints
    plot(0,0,'o','MarkerFaceColor','w','MarkerSize',7)
    plot(ya(i),za(i),'o','MarkerFaceColor','w','MarkerSize',7)    
       
    % Plot CoMs (uncomment to plot)
    plot(y1(i),z1(i),'ko','MarkerFaceColor','w','MarkerSize',5)
    plot(y1(i),z1(i),'kx','MarkerSize',5)
    plot(y2(i),z2(i),'ko','MarkerFaceColor','w','MarkerSize',5)
    plot(y2(i),z2(i),'kx','MarkerSize',5)
    
    plot(yb(1:i),zb(1:i))


    %-------------------------------------------------------------------------%
    % Print Time
    %-------------------------------------------------------------------------%
    x_time = -0.8;
    y_time = 0.8;

    label = num2str(tout(i));
    text(x_time, y_time, 'Time (s):', 'Color', 'k', 'FontSize', 16,...
        'fontWeight', 'normal');
    text(x_time, y_time - 0.2, label, 'Color', 'k', 'FontSize', 16);
    
    
    % Axes
    axis equal
    axis([-1.5 +1.5 -1.5 1.5])
    % grid on
       
    drawnow
    
    hold off
    %tout(i)

end
