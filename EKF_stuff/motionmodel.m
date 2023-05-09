
function [xstate1] = motionmodel(xstate,Vin,Dn,t)
% This function takes in the current state of a system, the control inputs Vin
% and disturbance inputs Dn, and the time elapsed t, and returns the updated
% state of the system based on a simple motion model.


% Update the x,y position of the system using the current x position, 
% the linear velocity Vin(1) and disturbance Dn(1) in the x direction, 
% and the time elapsed t, all scaled by the cos/sin of the current orientation.
xstate1(1)=xstate(1)+(Vin(1)+Dn(1))*t*cos(xstate(3));
xstate1(2)=xstate(2)+(Vin(1)+Dn(1))*t*sin(xstate(3));

% Update the orientation of the system using the current orientation,
% the angular velocity Vin(2) and disturbance Dn(2), and the time elapsed t.
xstate1(3)=xstate(3)+(Vin(2)+Dn(2))*t;


end
