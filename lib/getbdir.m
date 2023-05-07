function a= getbdir(B,P,th)
%GETBDIR Summary of this function goes here
%   Detailed explanation goes here
% B- beacon coordinates
% P- current position
% th- direction of motion in the global frame
% a - angle <B,P,Q> mod 180º


var=B-P;

b_ang=atan2(var(2),var(1))-th+0.0175*randn;

a=mod(b_ang,pi);









end

