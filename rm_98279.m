function rm_98279(varargin)


addpath lib/
clear all;close all;clc;

if nargin < 3
    plotting = 1;
else
    plotting = varargin{3};
end

if nargin < 2
    Dt = 1;
else
    Dt = varargin{2};
end

if nargin < 1
    N = 4;
else
    N = varargin{1};
end


% plotting=input('plots yes(1) or no(0)?');
if ~(plotting == 0 || plotting == 1)
    warning('Input should be 0 or 1.');
end
%for testing plotting=1



%% initialização values
% posição inicial robot
x=[0,0,0];

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

[xstate_EKF]=ekf_calculations(landmarkxy,control_input_mea,control_input_true,obs_range_bearing,xstate_true,obs_landmark_ID,[Vn;Wn],[sig_r;sig_phi],plotting);


%--------------------------------------------------------------------------


[dd,tri]=plotting_velocities_wheels(xinterp,yinterp,Dt,r,L,Vn,Wn,plotting);


%--------------------------------------------------------------------------


DIR = 'output';
% Check if the directory exists
if ~exist(DIR, 'dir')
    % Create the directory if it does not exist
    mkdir(DIR)
end

% Save the files to the directory
disp(['Saving all files (loc_98279.txt, DD_98279.txt, TRI_98279.txt) to ' DIR ' folder']);
save(fullfile(DIR, 'loc_98279.txt'), 'xstate_true', '-ascii');
save(fullfile(DIR, 'DD_98279.txt'), 'dd', '-ascii');
save(fullfile(DIR, 'TRI_98279.txt'), 'tri', '-ascii');





end


