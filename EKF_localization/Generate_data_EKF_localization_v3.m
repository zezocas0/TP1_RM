% ===============================================================================================
% Encyclopedia of EEE -- Robot Localization: An Introduction
%
%           Shoudong Huang, Gamini Dissanayake
%
%           Centre for Autonomous Systems
%           Faculty of Engineering and Information Technology
%           University of Technology, Sydney
%           NSW 2007, Australia
% 
% MATLAB code for the examples: Version: 1.0
% ===============================================================================================
% 
% Copyright (C) 2016 Shoudong Huang 
% University of Technology, Sydney, Australia
% 
% Author:  Shoudong Huang     -- Shoudong.Huang@uts.edu.au
%          
% Please contact Shoudong Huang {Shoudong.Huang@uts.edu.au} if you have any questions/comments about the code.
%
% Modified by v.santos (vitor@ua.pt, March 2023): Version 3.0
% ===============================================================================================
% 
%% Generate data and perform EKF localization
%  this code generates data for EKF localization and then run EKF
%
% Data generated include: 
% landmark data -- landmarkxy
% control data -- control_input_mea
% observation data -- obs_range_bearing
% true robot pose data -- xstate_true (for comparison)
%
% Shoudong Huang, 2016 April
%
clc
clear all
close all

% set this value to 1 to use a circular trajectory with 100 steps
test100=1;

if test100==1
    fDir='data100/';
else
    fDir='data/';
end
try
rmdir(fDir,'s') %erase the existing directory to generate new data
catch
end
mkdir(fDir); %and create a fresh one :-)

%% noise level setting -- to generating data (must be the same when running the EKF)
%control: velocity, turnrate
sig_v     = 0.1;
sig_omega = 0.1;

% observation: range, bearing
sig_r   = 0.1;
sig_phi = 0.1;

%% landmark setting: 4 landmarks, format: ID, x, y
landmarkxy = [
    -3 -3  0
    2 2 -2
    3 4 0
    4 4 -2
    ];
%% save the landmark data in a file
save(fDir + "landmarkxy.mat", 'landmarkxy')


%% control inputs, format: time_step, linear velocity, turnrate (angular velocity)
control_input_true = [
    0 1   0
    1 0.7 pi/30
    2 1.1 pi/30
    3 1   0
    ];

% number of motion steps (starts from time step 0)
num_steps = height(control_input_true);

% in case you are generating a longer circular trajectory with constant velocity
if test100==1
    num_steps=100;
    control_input_true = zeros(num_steps,3);
    for i=1:num_steps
        control_input_true(i,:)=[i-1, 1, 2*pi/num_steps]; %[ID, linear vel., ang velo.]
    end
end

%% Add noise to the true control inputs and save it

%generating measured control inputs by adding noises (for EKF to use)
control_input_mea     =control_input_true;
%control noises
noises_v              =randn(num_steps,1)*sig_v;
noises_omega          =randn(num_steps,1)*sig_omega;
control_input_mea(:,2)=control_input_mea(:,2)+noises_v;
control_input_mea(:,3)=control_input_mea(:,3)+noises_omega;
% save the control data
save(fDir + "control_input_mea.mat", 'control_input_mea')

%% generate ground true robot poses
% format: pose ID, x, y, phi
xstate_true = [0, zeros(1,3)]; % pose at time 0

for i=1:num_steps
    control_i = control_input_true(i,2:3);
    control_noise = [0;0];
    Delta_T = 1;  %sampling time
    xstatet1 = motionmodel(xstate_true(end,2:4),control_i,control_noise,Delta_T);
    xstate_true = [xstate_true; i xstatet1];  %accumulate the xstate Ground Truth
end
% save xstate_true for comparison
save(fDir + "xstate_true.mat",'xstate_true')

%% generating observation data
% the observed landmark ID at each time step
% format: time_step ID1 ID2 (assume always see two landmarks for simplication)
obs_landmark_ID = [
    1 1 2  %beacons 1 and 2
    2 1 2  %beacons 1 and 2
    3 1 3  %beacons 1 and 3
    4 3 4  %beacons 3 and 4
    ];

% use alternative longer circular trajecotry
if test100==1
    obs_landmark_ID =  zeros(num_steps,3);
    for i=1:num_steps
        obs_landmark_ID(i,:)=[i, 1, 2];
    end
end
%% range and bearing observations
% format: time_step ID1 r1 phi1 ID2 r2 phi2
obs_range_bearing = [];
for i=1:num_steps
    landmark1=landmarkxy(obs_landmark_ID(i,2),2:3);
    landmark2=landmarkxy(obs_landmark_ID(i,3),2:3);
    
    % observation noises
    noise_r      =randn*sig_r;
    noise_phi    =randn*sig_phi;
    sensor_noise =[noise_r  noise_phi];
    
    % range-bearing to one landmark
    z1 = sensormodel(landmark1, xstate_true(i+1,2:4), sensor_noise);
    
    % observation noises
    noise_r=randn*sig_r;
    noise_phi=randn*sig_phi;
    sensor_noise = [noise_r  noise_phi];
    % range-bearing to another landmark
    z2 = sensormodel(landmark2,xstate_true(i+1,2:4),sensor_noise);
    % store the obs data
    obs_range_bearing = [obs_range_bearing
                         i obs_landmark_ID(i,2) z1 obs_landmark_ID(i,3) z2
                        ];
end

%obs_range_bearing
save(fDir + "obs_range_bearing.mat",'obs_range_bearing')


