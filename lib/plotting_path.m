function [total_x,total_y,xinterp,yinterp] = plotPath(B, Vn, Dt)


figure(1);

set(1, 'Position', [1000,0, 600, 400])

hold on;
title("direct path and pchip path")

total_steps=[];

% plot the dots in between the beacons
for i = 1:length(B) % loop through each beacon
    % get the current beacon
    curr_beacon = B(i);
    

    if i == 1

        % from 0,0 to the first beacon
        steps = round(sqrt((curr_beacon.X)^2 + (curr_beacon.Y)^2) / (Vn*Dt));   

        total_steps=[total_steps ;steps];
        x=linspace(0, curr_beacon.X, steps);
        total_x=x;
        y=linspace(0, curr_beacon.Y, steps);
        total_y=y;


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





% plot the interpolated curve
plot( xinterp,yinterp, 'g-','LineWidth',2);

hold on;
    %plotting the stepped xy values
plot(total_x,total_y,'r.','MarkerSize',10);

hold on;
%plot the beacons
for i=1:length(B)
    plot(B(i).X,B(i).Y,'bo','LineWidth',2,'MarkerSize',10);
end

for i = 1:length(xinterp)
    plot([xinterp(i), xinterp(i)], [0, yinterp(i)], '--k','Color','c');
end

% set the axis limits and labels
ymax=   max(yinterp);
xmax = B(end).X; % set the maximum x-coordinate to the last beacon in the array
axis([0-10 xmax+10 0-10 ymax+10]);

xlabel('X');
ylabel('Y');
legend('pchip ','stepped path','beacons','Location','northwest')






end