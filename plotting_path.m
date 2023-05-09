function [total_x,total_y,xinterp,yinterp] = plotPath(B, Vn, Dt,plotting)


figure;
subplot(1, 2, 1); 
title('path to each beacon ')
hold on;


total_steps=[];

% plot the dots in between the beacons
for i = 1:length(B) % loop through each beacon
    % get the current beacon
    curr_beacon = B(i);
    
    % plot the current beacon as a blue circle
    plot(curr_beacon.X, curr_beacon.Y, 'bo', linewidth=2,MarkerSize=10);

    % plot the line from (0,0) to the first beacon
   
    if i == 1

        % from 0,0 to the first beacon
        steps = round(sqrt((curr_beacon.X)^2 + (curr_beacon.Y)^2) / (Vn*Dt));   
        total_steps=[total_steps ;steps];
        x=linspace(0, curr_beacon.X, steps);
        total_x=x;
        y=linspace(0, curr_beacon.Y, steps);
        total_y=y;

        hold on; 
        plot(x,y, 'r*', 'MarkerSize', 10);

    else
        % get the previous beacon
        prev_beacon = B(i-1);

        % set the range of the plot to the x-coordinates between the two beacons
        xmin = prev_beacon.X;
        xmax = curr_beacon.X;
        ymin = prev_beacon.Y;
        ymax = curr_beacon.Y;

        % calculate the number of steps based on the distance and velocity
        steps = round(sqrt((xmax-xmin)^2 + (ymax-ymin)^2) / (Vn*Dt));
        total_steps=[total_steps ;steps];
        
        % create a vector of uniformly spaced points along the x-axis
        x = linspace(xmin, xmax, steps);
        total_x=[total_x ,x];
        % create a vector of uniformly spaced points along the y-axis
        y = linspace(ymin, ymax, steps);
        total_y=[total_y ,y];
        % plot the dots in between the beacons
        
        hold on;
        plot(x, y, 'r*', 'MarkerSize', 10);
        
    end
end

% set the axis limits and labels

xmax = B(end).X; % set the maximum x-coordinate to the last beacon in the array
axis([0-10 xmax+10 0-10 ymax+10]);
xlabel('X');
ylabel('Y');

% create a subplot with pchip interpolation

%beacon and 0,0
bn0points=[0 0];
for i=1:length(B)
    bn0points=[bn0points; B(i).X B(i).Y];
end


xinterp=[];
for i=1:length(B)
    
    x_interp=linspace(bn0points(i,1),bn0points(i+1,1),total_steps(i)+1);
    xinterp=[xinterp x_interp];

    
end



%remove duplicates

[Z,idx]= unique(xinterp','stable');
xinterp=xinterp(:,idx);


% interpolate the y-coordinates using pchip()
yinterp = pchip(bn0points(:,1),bn0points(:,2),xinterp );





subplot(1, 2, 2);
title('path using pchip')
% plot the interpolated curve
plot( xinterp,yinterp, 'g-','LineWidth',2);

hold on;
for i = 1:length(xinterp)
    plot([xinterp(i), xinterp(i)], [0, yinterp(i)], '--k','Color','c');
end
hold on;
for i=1:length(B)
    %plot the beacons
    plot(B(i).X,B(i).Y,'bo','LineWidth',2,'MarkerSize',10);
end



% set the axis limits and labels
axis([0-10 xmax+10 0-10 ymax+10]);
xlabel('X');
ylabel('Y');

if plotting== 0
    close all;
end





end