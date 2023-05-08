%  trabalho pratico 1 
% 
% José Santos--98279
% 


addpath lib/
close all; clear all; clc;
%% initialização values
% posição inicial robot
P=[0,0,0];
model=1; % 1:dd ; 2 :tricicle
N=4; % number of beacons
Dt=1; % s 
r=0.15; %m wheel radius
L=1; %m wheel seperation
Vn=0.1; %uncertainty of linear velocity 0.1m/s
Wn=0.1; %uncertainty of angular velocity 0.1m/s

Dt=5;
Vn=2;
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




% calculate and plot the path to each beacon 

[total_x,total_y,x_interp,y_interp] = plotting_path(B, Vn, Dt,1);


%init robot
robot_model = robotModel(model,3);

%plot robot  ,xinterp,yinterp and beacons

figure(2)
plot(x_interp,y_interp,'r*')
hold on;
for i=1:length(B)
    plot(B(i).X,B(i).Y,'bo','MarkerSize',10,'LineWidth',2)
end
% hold on; 
% plot(robot_model(1,:),robot_model(2,:),'g','MarkerSize',5,'LineWidth',2)


%initial testing, make robot move to the first beacon following x_interp and y_interp values
% passage_points=[x_interp;y_interp];   
% [hr]=InvMovementDD(passage_points,robot_model,length(x_interp),1);


