%  trabalho pratico 1 
% 
% José Santos--98279
% 


addpath lib/
addpath EKF_stuff/
close all; clear all; clc;

 %% inputs for user 
% plotting=input('plots yes(1) or no(0)?');
% if ~(plotting == 0 || plotting == 1)
%     warning('Input should be 0 or 1.');
% end
%for testing plotting=1
plotting=0;



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



%init robot
robot_model = robotModel(model,3);

if plotting==1

    figure(2)
    plot(xinterp,yinterp,'r*')
    hold on;
    for i=1:length(B)
        plot(B(i).X,B(i).Y,'bo','MarkerSize',10,'LineWidth',2)
    end
    hold on; 
    robot=fill(robot_model(1,:),robot_model(2,:),'g');
    % plot(robot_model(1,:),robot_model(2,:),'g','MarkerSize',5,'LineWidth',2)


end

% velocities of the robot when moving between the points interp(oints)
[v,w,theta,dtheta]=plotting_velocities(xinterp,yinterp,Dt,r,L);

%% EKF CALCULATIONS -------------------------------------------------------





landmarkxy=[];
for i=1:length(B)
    landmarkxy= [landmarkxy;i, B(i).X,B(i).Y];
end


% --------------------------------------------------------------------------
%% control inputs, format: time_step, linear velocity, turnrate (angular velocity)

for i=1:length(v)
    control_input_true(i,:) = [i v(i) w(i)];
end

% number of motion steps (starts from time step 0)
num_steps = height(control_input_true);

%% Add noise to the true control inputs and save it

%generating measured control inputs by adding noises (for EKF to use)
control_input_mea     =control_input_true;
%control noises
noises_v              =randn(num_steps,1)*sig_v;
noises_omega          =randn(num_steps,1)*sig_omega;

control_input_mea(:,2)=control_input_mea(:,2)+noises_v;
control_input_mea(:,3)=control_input_mea(:,3)+noises_omega;
% save the control data


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

%% generating observation data
% the observed landmark ID at each time step
% format: time_step ID1 ID2 (assume always see two landmarks for simplication)
obs_landmark_ID = [];
for i=1:length(xinterp)-1
    % randomly choose two landmarks
    landmark_ID = randperm(N,2);
    obs_landmark_ID = [obs_landmark_ID
                       i landmark_ID
                      ];
end

%observation range bearing(to put in function later)
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







% --------------------------------------------------------------------------







% for storing the results
xstate_EKF = [0, zeros(1,3)]; % pose at time 0
P_EKF      = 0.01*eye(3);  % initial covariance matrix

% noise
Q=[Vn^2,0;0,Wn^2];

R_i=diag([sig_r^2,sig_phi^2]);



%% initial estimate in EKF
for step = 1:num_steps
    
    disp('Running step');
    disp(step);
    disp('------------------------------------------------');
    % get the data needed for one-step EKF
    % EKF estimate at time t
    xstate_t = xstate_EKF(end,2:4)';
    P_t = P_EKF(end-2:end,:);

    % control input at time t
    control_t= control_input_mea(step,2:3);
    % observation data at time t+1
    obs_t1 = obs_range_bearing(step,2:end);
    
    %discretization time interval
    Delta_T=1;
    R = [R_i,zeros(2,2);zeros(2,2),R_i]; % because observing two landmarks each step
    %using ekf function
    [xstateT1_T1,PT1_T1] = ekf(xstate_t,P_t,control_t,obs_t1,landmarkxy,Delta_T,Q,R);
    
    %update
    xstate_EKF = [xstate_EKF; step, xstateT1_T1];
    P_EKF = [P_EKF; PT1_T1];
end

%% comparing results. calculating error
%xstate_EKF=ID,x,y,phi
%xstate_true=ID,xinterp,yinterp,theta(from velocity calculations)
error_state=xstate_EKF-xstate_true



%% draw the estimated robot poses and uncertainty ellipses
