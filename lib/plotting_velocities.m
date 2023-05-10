function [v,w,theta,dtheta]=plotting_velocities(xinterp,yinterp,Dt,Vn,Wn,plotting)
%calculating velocities for EKF
%input: xinterp, yinterp pchip()results
% Dt = time step
% Vn = noise on linear velocitiy
% W = noise on angular velocity
% plotting =if you want to plot the velocities
%outputs linear vel(v) angular vel(w) and theta and dtheta

steps=length(xinterp);
% velocities and stuff
for i=2:steps
    distance(i) = sqrt((xinterp(i)-xinterp(i-1))^2+(yinterp(i)-yinterp(i-1))^2);
    
    % theta(i)= acos((xinterp(i)-xinterp(i-1))/distance(i));
    % if yinterp(i)<yinterp(i-1)
    %     theta(i)=-theta(i);
    % end
    theta(i)=asin((yinterp(i)-yinterp(i-1))/distance(i));


end
for i=1:steps-1
    dtheta(i) = theta(i+1)-theta(i);
end
% calculate velocities and angular velocities for each model

    v_noise=Vn*randn(length(dtheta),1);
    w_noise=Wn*randn(length(dtheta),1);



for i=1:steps-1
    %calculations * (1+noise)


    v(i) = distance(i)/Dt* (1+v_noise(i));
    w(i) = dtheta(i)/Dt *(1+w_noise(i)) ;

end

if plotting==1

    figure(2)
    set(2, 'Position', [1000,600, 600, 400])

    subplot(2,1,1)
    plot(v)
    title('Velocity')
    subplot(2,1,2)
    plot(w)
    title('Angular Velocity')
end


end