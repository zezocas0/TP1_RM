function [dd,tri]=plotting_velocities(xstate_EKF,Dt,r,L,Vn,Wn)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            funciton plotting_velocities
%   This function calculates the velocities and angular velocities for DD and TRI models
%   Created by: José Santos 98279
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% calculate velocities and angular velocities for each model

for i=1:length(xstate_EKF)
    x_ekf(i)=xstate_EKF(i,2);
    y_ekf(i)=xstate_EKF(i,3);
    t_ekf(i)=xstate_EKF(i,4);

end


% noise of angular and linear velocity
v_noise=Vn*randn(length(xstate_EKF),1);
w_noise=Wn*randn(length(xstate_EKF),1);

%calculate the generic velocities based on x,y and theta from EKF

for i=2:length(xstate_EKF)
    distance(i) = sqrt((x_ekf(i)-x_ekf(i-1))^2+(y_ekf(i)-y_ekf(i-1))^2);
    theta(i)=atan2((y_ekf(i)-y_ekf(i-1)),(x_ekf(i)-x_ekf(i-1)));

    %velocities +noise   
    v(i)=distance(i)/Dt* (1+v_noise(i)); 
    w(i)=(theta(i)-theta(i-1))/Dt* (1+w_noise(i));
end


for i=1:length(v)
    % DD - left and right angular velocity

    DD_l(i)= (v(i)-(w(i)*L/2))/r;
    DD_r(i)= (v(i)+(w(i)*L/2))/r;


    %TRI- rear wheel velocity and front wheel angular position


    TRI_v(i)= v(i)-(w(i)*L);
    

    %calculate angular velocity of rear wheel
    w_r(i)=w(i)*L;

    % %front wheel direction
    f_alfa(i)=atan(w_r(i)/TRI_v(i));
    
end
% f_alfa=asin(w*L./TRI_v)-- gives imaginary numbers

% values to save on main function
dd=[DD_l;DD_r];
tri=[TRI_v;f_alfa];



%plotting
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

subplot(3,1,3)
plot (1:length(f_alfa),f_alfa,'b')
hold on
title('tricycle angle')
xlabel('Time')
ylabel('Angular Velocity')


end