function [dd,tri]=plotting_velocities(xinterp,yinterp,Dt,r,L,Vn,Wn,plotting)
% this function plots the velocities and angular velocities for the
% tricycle and differential drive models
% inputs: xinterp and yinterp are the interpolated x and y values from the
% path
% outputs: dd and tri are the velocities and angular velocities for the
% differential drive and tricycle models


% velocities and stuff
for i=2:length(xinterp)
    distance(i) = sqrt((xinterp(i)-xinterp(i-1))^2+(yinterp(i)-yinterp(i-1))^2);
    theta(i) = atan2(yinterp(i), xinterp(i));
    
end
for i=1:length(theta)-1
    dtheta(i) = theta(i+1)-theta(i);
end
% calculate velocities and angular velocities for each model

for i=1:length(dtheta)
    % dd

    v_dd(i) = distance(i)/Dt;
    w_dd(i) = (dtheta(i)/Dt)*(r/L);

    %tricicle 

    v_tri(i) = distance(i)/Dt;
    w_tri(i) = (dtheta(i)/Dt)*(v_tri(i)/L)*tan(theta(i));


end

dd=[v_dd;w_dd]';
tri=[v_tri;w_tri]';
% plot the velocities and angular velocities
x=1:length(v_dd);
figure;
subplot(2,2,1);
plot(x,v_dd);
title('linear velocity for dd');
subplot(2,2,2);
plot(x,w_dd);
title('angular velocity for dd');

subplot(2,2,3);
plot(x,v_tri);
title('linear velocity for tri');
subplot(2,2,4);
plot(x,w_tri);
title('angular velocity for tri');




