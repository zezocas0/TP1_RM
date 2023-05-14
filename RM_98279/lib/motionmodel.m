
function [xstate1] = motionmodel(xstate,Vin,Dn,t)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            funciton motionmodel
%   FUnction NOT modified. obtained by teacher,
%   only added coments
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Update the x,y position of the system using the current x position, 
% the linear velocity Vin(1) and disturbance Dn(1) in the x direction, 
% and the time elapsed t, all scaled by the cos/sin of the current orientation.
xstate1(1)=xstate(1)+(Vin(1)+Dn(1))*t*cos(xstate(3));
xstate1(2)=xstate(2)+(Vin(1)+Dn(1))*t*sin(xstate(3));

xstate1(3)=xstate(3)+(Vin(2)+Dn(2))*t;


end
