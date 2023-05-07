function [VR,VL]= inkinDD(X,Y,th,L,t)
% X - target position in x (meters)
% Y - target position in y (meters)
% th- target orientation in th (radians)
% L - wheel separation (meters)
% t - time to accomplish the trajectory (seconds)
% Notice: X, Y and th can not be set all at the same time
% one of them must be NaN for the function to return valid results

if th ==0
V=x/t;
VL=V;VR=V;
return
end

w=th/t;

if isnan(X)
    V=X*w/(sin(th));
elseif isnan(Y)
    V=X*w/(sin(th));
else
        disp('Error: X, Y and th can not be set all at the same time')
        disp('one of them must be NaN for the function to return valid results')
end

VR= V+L*w/2;
VL= V-L*w/2;







end

