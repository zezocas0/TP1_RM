function [VR,VL]=invkinDDxy(X,Y,L,t,Xn,Yn)
% X - target position in x (meters)
% Y - target position in y (meters)
% L - wheel separation (meters)
% t - time to accomplish the trajectory (seconds)
% Xn - next target position in x (meters)
% Yn - next target position in y (meters)


if X==0 && Y==0 % perform rotation
       %calculate w 
       phi=atan2(Yn,Xn);
       w=phi/t;
       VR=w*L/2;
       VL=-VR;
else 
    D=norm([X Y]);
    V=D/t;
    VL=V;VR=V;
end
