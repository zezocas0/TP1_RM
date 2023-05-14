function [v,w,theta,dtheta]=plotting_velocities(xinterp,yinterp,Dt,Vn,Wn)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            funciton plotting_velocities
%   Function to calculate the linear and angular velocities
%   Created by: José Santos 98279
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%outputs velocities, theta and dtheta

steps=length(xinterp);
% velocities and stuff
for i=2:steps
    distance(i) = sqrt((xinterp(i)-xinterp(i-1))^2+(yinterp(i)-yinterp(i-1))^2);
    theta(i)=asin((yinterp(i)-yinterp(i-1))/distance(i));
end

for i=1:steps-1
    dtheta(i) = theta(i+1)-theta(i);
end
% calculating noise for velocities
    v_noise=Vn*randn(length(dtheta),1);
    w_noise=Wn*randn(length(dtheta),1);

for i=1:steps-1
    %calculations * (1+noise)


    v(i) = distance(i)/Dt* (1+v_noise(i));
    w(i) = dtheta(i)/Dt *(1+w_noise(i)) ;

end


%plotting velocities 
figure(2)
set(2, 'Position', [1000,600, 600, 400])
subplot(2,1,1)
plot(v)
title('Linear Velocity')
subplot(2,1,2)
plot(w)
title('Angular Velocity')

end