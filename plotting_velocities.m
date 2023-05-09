function [v,w,theta,dtheta]=plotting_velocities(xinterp,yinterp,Dt,r,L)
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

    v(i) = distance(i)/Dt;
    w(i) = dtheta(i)/Dt;

end


