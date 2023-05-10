%  trabalho pratico 1 
% 
% José Santos--98279
% 


addpath lib/
close all; clear all; clc;

 %% inputs for user 
% plotting=input('plots yes(1) or no(0)?');
% if ~(plotting == 0 || plotting == 1)
%     warning('Input should be 0 or 1.');
% end
%for testing plotting=1
plotting=1;



%% initialização values
% posição inicial robot
x=[0,0,0];

model=1; % 1:dd ; 2 :tricicle
N=4; % number of beacons
Dt=1; % s 
r=0.15; %m wheel radius
L=1; %m wheel seperation
Vn=0.1; %uncertainty of linear velocity 0.1m/s
Wn=0.1; %uncertainty of angular velocity 0.1m/s

% Dt=5;
Vlinear=5;



%% noise level setting -- to generating data for ekf
%control: velocity, turnrate
sig_v     = 0.1;
sig_omega = 0.1;

% observation: range, bearing
sig_r   = 0.1;
sig_phi = 0.1;



%% robot and beacon initialization
% calling the beacon detection
% function [B]=BeaconDetection(N,P,obsNoise)
% INPUTS:
% N - number of beacons to create/use (N>3) but large values may not be respected
% P - current estimated position (x,y,a). (0,0,0) if absent.
% obsNoise - observation noise [range, heading]. If not passed, use a default value

B=BeaconDetection(N,x);

% B - array of structures with data from beacons:
% B.X - real beacon X position (fixed and known)
% B.Y - real beacon Y position (fixed and known)
% B.d - measured distance (with uncertainty)
% B.a - measured angle (with uncertainty)
% B.dn - sigma in B.d (either the passed in obsNoise or a default)
% B.an - sigma in B.a (either the passed in obsNoise or a default)




%% calculate and plot the path to each beacon 

[total_x,total_y,xinterp,yinterp] = plotting_path(B, Vlinear, Dt,plotting);




% velocities of the robot when moving between the points interp(oints)
[v,w,theta,dtheta]=plotting_velocities(xinterp,yinterp,Dt,Vn,Wn,plotting);



%% EKF data generation  -------------------------------------------------------

[xstate_true,control_input_true,control_input_mea,obs_range_bearing,landmarkxy,obs_landmark_ID]=generate_ekf_data(N,sig_v,sig_omega,sig_r,sig_phi,Dt,[v;w],theta,[xinterp;yinterp]);



% --------------------------------------------------------------------------

%% EKF calculations ---------------------------------------------------------

[error_state]=ekf_calculations(landmarkxy,control_input_mea,control_input_true,obs_range_bearing,xstate_true,obs_landmark_ID,[Vn;Wn],[sig_r;sig_phi],plotting);



%--------------------------------------------------------------------------



[dd,tri]=plotting_velocities_wheels(xinterp,yinterp,Dt,r,L,Vn,Wn,plotting);
