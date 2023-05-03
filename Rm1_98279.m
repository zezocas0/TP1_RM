%  trabalho pratico 1 
% 
% José Santos--98279
% 



%% initialização values
% posição inicial robot
P=[0,0,0];

N=4; % number of beacons
Dt=1; % s 
r=0.15; %m wheel radius
L=1; %m wheel seperation
Vn=0.1; %uncertainty of linear velocity 0.1m/s
Wn=0.1; %uncertainty of angular velocity 0.1m/s

%% robot and beacon initialization

% calling the beacon detection
% function [B]=BeaconDetection(N,P,obsNoise)
% INPUTS:
% N - number of beacons to create/use (N>3) but large values may not be respected
% P - current estimated position (x,y,a). (0,0,0) if absent.
% obsNoise - observation noise [range, heading]. If not passed, use a default value

B=BeaconDetection(N,P);

% B - array of structures with data from beacons:
% B.X - real beacon X position (fixed and known)
% B.Y - real beacon Y position (fixed and known)
% B.d - measured distance (with uncertainty)
% B.a - measured angle (with uncertainty)
% B.dn - sigma in B.d (either the passed in obsNoise or a default)
% B.an - sigma in B.a (either the passed in obsNoise or a default)


% initialization of robot.
%  1 or 0 if to see which robot to implement 1 if Diff drive, 0 if tricycle






% calculate and plot the path to each beacon 





%for loop for the kalman filter and movement of the robot, knowing it
%starts at 0,0






