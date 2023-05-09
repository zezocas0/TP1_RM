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



% %init robot
% robot_model = robotModel(model,3);

% if plotting==1

%     figure(2)
%     plot(xinterp,yinterp,'r*')
%     %beacons locations
%     hold on;
%     for i=1:length(B)
%         plot(B(i).X,B(i).Y,'bo','MarkerSize',10,'LineWidth',2)
%     end
%     hold on; 
%     robot=fill(robot_model(1,:),robot_model(2,:),'g');
%     % plot(robot_model(1,:),robot_model(2,:),'g','MarkerSize',5,'LineWidth',2)


% end

% velocities of the robot when moving between the points interp(oints)
[v,w,theta,dtheta]=plotting_velocities(xinterp,yinterp,Dt,Vn,Wn,plotting);

%% EKF CALCULATIONS -------------------------------------------------------





landmarkxy=[];
for i=1:length(B)
    landmarkxy= [landmarkxy;i, B(i).X,B(i).Y];
end


% --------------------------------------------------------------------------
%% control inputs, format: time_step, linear velocity, turnrate (angular velocity)
control_input_true=[];
for i=1:length(v)
    control_input_true = [control_input_true;i, v(i) ,w(i)];
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
xstate_true = [0, xinterp(1), yinterp(1), 0]; % pose at time 0

for i=1:num_steps
    control_i = control_input_true(i,2:3);
    control_noise = [0;0];
    %sampling time
    %xinterp, yinterp and theta to be used for the motion model
    xstatet1 = motionmodel([xinterp(i),yinterp(i),theta(i)],control_i,control_noise,Dt);
    xstate_true = [xstate_true; i xstatet1]; %accumulate the xstate Ground Truth
end


%% generating observation data
% the observed landmark ID at each time step
% format: time_step ID1 ID2 (assume always see two landmarks for simplication)
obs_landmark_ID = [];
for i=1:length(xinterp)-1
    % randomly choose two landmarks
    landmark_ID = randperm(N,2);
    obs_landmark_ID = [obs_landmark_ID;i landmark_ID];
end

%observation range bearing(to put in function later)
obs_range_bearing = [];
obs_range_bearing_old=[];
for i=1:num_steps

    id1=obs_landmark_ID(i,2);
    id2=obs_landmark_ID(i,3);
   
    % observation noises
    noise_r      =randn*sig_r;
    noise_phi    =randn*sig_phi;
    sensor_noise =[noise_r  noise_phi];
    

    landmark1=landmarkxy(obs_landmark_ID(i,2),2:3);
    landmark2=landmarkxy(obs_landmark_ID(i,3),2:3);    
    % range-bearing to one landmark
    z1_old = sensormodel(landmark1, xstate_true(i+1,2:4), sensor_noise);
    % range-bearing to another landmark
    z2_old = sensormodel(landmark2,xstate_true(i+1,2:4),sensor_noise);
    
    % store the obs data
    obs_range_bearing_old = [obs_range_bearing_old;
                         i obs_landmark_ID(i,2) z1_old obs_landmark_ID(i,3) z2_old];


    B_bearing=BeaconDetection(N,xstate_true(i+1,2:4)); 

    % range-bearing to one landmark
    z1 =[B_bearing(id1).d B_bearing(id1).a] +sensor_noise;
    % range-bearing to another landmark
    z2=[B_bearing(id2).d B_bearing(id2).a]+sensor_noise;
    
    % store the obs data
    obs_range_bearing = [obs_range_bearing
                         i obs_landmark_ID(i,2) z1 obs_landmark_ID(i,3) z2];
    
    %print line where isnan happened
    if any(isnan(obs_range_bearing(:)))
        disp(i)
    end
    % if obs_range_bearing has any value that is NaN,replace the Nan with 0
    if any(isnan(obs_range_bearing(:)))
        obs_range_bearing(isnan(obs_range_bearing))=0;
    end
end

% --------------------------------------------------------------------------







% for storing the results
xstate_EKF = [0, zeros(1,3)]; % pose at time 0
P_EKF      = 0.01*eye(3);  % initial covariance matrix

% noise
Q=[Vn^2,0;0,Wn^2];

R_i=diag([sig_r^2,sig_phi^2]);


disp("EKF running...         ")

%% initial estimate in EKF
for step = 1:num_steps
    
    
	fprintf(1,'\b\b\b\b\b\b %.2d %%',floor(step/num_steps*100))%progress status


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
fprintf("EKF done!        \n\n ")

%% comparing results. calculating error
%xstate_EKF=ID,x,y,phi
%xstate_true=ID,xinterp,yinterp,theta(from velocity calculations)
%error_state=ID,error_x,error_y,error_phi
error_state=xstate_EKF(:,2:4)-xstate_true(:,2:4);
%Adding ID
error_state=[xstate_EKF(:,1),error_state];



%% draw the estimated robot poses and uncertainty ellipses
% close all; %close everything else, for testing

arrow_length=1;
if plotting==1

    figure(5)
    hold on;

    %beacons locations
    plot(landmarkxy(:,2),landmarkxy(:,3),'bo','MarkerSize',10,'LineWidth',2)
    

    disp("creating the plot ...        ")    
    %plotting  the reason for making all this callculations!
    for i=1:num_steps

	fprintf(1,'\b\b\b\b\b %.2d %%',floor(i/num_steps*100))%progress status
    
    %xy covariance
    Pxy=P_EKF(3*i:3*i+1,1:2);

    ekf_x=xstate_EKF(i+1,2);
    ekf_y=xstate_EKF(i+1,3);

    % CV=GetCov(Pxy,ekf_x,ekf_y);  % by wangzhan, make it large on purpose, not now
    % plot(CV(1,:),CV(2,:),'-b');

    plot(xstate_EKF(i+1,2),xstate_EKF(i+1,3),'bo','linewidth',2);
    

    % draw the robot heading
    dy = arrow_length*sin(xstate_EKF(i+1,4));
    dx = arrow_length*cos(xstate_EKF(i+1,4));
    quiver(xstate_EKF(i+1,2),xstate_EKF(i+1,3),...
           dx, dy, 0, 'Color', 'b','linewidth',1.2)
    
    %draw the true robot poses for comparison
    
    plot(xstate_true(i+1,2),xstate_true(i+1,3),'ro','linewidth',2);
    
    dx = arrow_length*cos(xstate_true(i+1,4));
    dy = arrow_length*sin(xstate_true(i+1,4));
    quiver(xstate_true(i+1,2),xstate_true(i+1,3),...
           dx, dy, 0, 'Color', 'r','linewidth',1.2)



    end
    fprintf("plot done!        \n\n ")


end