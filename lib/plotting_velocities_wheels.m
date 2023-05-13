function [dd,tri]=plotting_velocities(xinterp,yinterp,v,w,Dt,r,L,Vn,Wn,plotting)
% this function plots the velocities and angular velocities for the
% tricycle and differential drive models
% INPUTS:
% xinterp - x coordinates of the path
% yinterp - y coordinates of the path
% v - linear velocity
% w - angular velocity
% Dt - time step
% r - wheel radius
% L - wheel base
% Vn - noise on linear velocity
% Wn - noise on angular velocity
% plotting - 1 if you want to plot, 0 if not

% calculate velocities and angular velocities for each model
for i=1:length(v)
    dtheta(i)=atan2(yinterp(i+1)-yinterp(i),xinterp(i+1)-xinterp(i));

end


for i=1:length(v)
    % DD - left and right angular velocity
    % Left angular velocity = (linear velocity - (angular velocity x wheelbase/2)) / wheel radius

    % Right angular velocity = (linear velocity + (angular velocity x wheelbase/2)) / wheel radius    

    DD_l(i)= (v(i)-(w(i)*L/2))/r;
    DD_r(i)= (v(i)+(w(i)*L/2))/r;


    %TRI- rear wheel velocity and front wheel angular position

    % Rear wheel velocity = linear velocity - (angular velocity x wheelbase/2)
    % Front wheel angular position =dtheta



    TRI_w(i)= atan2(w(i), v(i) + L*tan(dtheta(i)));
    TRI_v(i)= v(i)-(w(i)*L/2);




end
dd=[DD_l;DD_r];
tri=[TRI_v;TRI_w];


if plotting==1
    figure
    subplot(3,1,1)
    plot(1:length(DD_l),DD_l,'b')
    hold on
    plot(1:length(DD_r),DD_r,'r')
    title('Differential Drive(velocities)')
    xlabel('Time')
    ylabel('Angular Velocity')
    legend('Left Wheel','Right Wheel')
    subplot(3,1,2)
    plot(1:length(TRI_v),TRI_v,'b')
    hold on
    title('Tricycle wheel vels')
    xlabel('Time')
    ylabel('Velocity')
    legend('Rear Wheel','Front Wheel')
    subplot(3,1,3)
    title('tricycle angle')
    plot (1:length(TRI_w),TRI_w,'b')
end

end