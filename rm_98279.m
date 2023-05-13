function rm_98279(N,Dt,r,L,Vn,Wn)


addpath lib/
close all;clc;
if nargin<6
    Wn=0.1; % angular velocity uncertainty
end
if nargin<5
    Vn=0.1; % velocity uncertainty
end
if nargin<4
    L=1; % wheel base
end
if nargin < 3
r=0.15; %m wheel radius
end
if nargin < 2
    Dt = 1; %time step
end
if nargin < 1
    N = 4; %number of beacons
end

plotting = 1;
% plotting=input('plots yes(1) or no(0)?');
if ~(plotting == 0 || plotting == 1)
    warning('Input should be 0 or 1.');
end



%% initialização values
% posição inicial robot
x=[0,0,0];
%velocidade linear do robot
Vlinear=5;

%% noise level setting -- to generating data for ekf
%control: velocity, turnrate
sig_v     = 0.1;
sig_omega = 0.1;
% observation: range, bearing
sig_r   = 0.1;
sig_phi = 0.1;




B=BeaconDetection(N,x);




%% calculate and plot the path to each beacon 
[total_x,total_y,xinterp,yinterp] = plotting_path(B, Vlinear, Dt);





% velocities of the robot when moving between the points interp(oints)
[v,w,theta,dtheta]=plotting_velocities(xinterp,yinterp,Dt,Vn,Wn,plotting);



%% EKF data generation  -------------------------------------------------------

[xstate_true,control_input_true,control_input_mea,obs_range_bearing,landmarkxy,obs_landmark_ID]=generate_ekf_data(N,sig_v,sig_omega,sig_r,sig_phi,Dt,[v;w],theta,[xinterp;yinterp]);


% --------------------------------------------------------------------------

%% EKF calculations ---------------------------------------------------------

[xstate_EKF]=ekf_calculations(landmarkxy,control_input_mea,control_input_true,obs_range_bearing,xstate_true,obs_landmark_ID,[Vn;Wn],[sig_r;sig_phi],plotting);


%--------------------------------------------------------------------------


[dd,tri]=plotting_velocities_wheels(xinterp,yinterp,v,w,Dt,r,L,Vn,Wn,plotting);


%--------------------------------------------------------------------------


DIR = 'output';
% Check if the directory exists
if ~exist(DIR, 'dir')
    % Create the directory if it does not exist
    mkdir(DIR)
end

% Save the files to the directory
disp(['Saving all files (loc_98279.txt, DD_98279.txt, TRI_98279.txt) to ' DIR ' folder']);


xstate_file=fopen(fullfile(DIR, 'loc_98279.txt'),'wt');
for i =1:length(xstate_EKF)
    fprintf(xstate_file,'%f ,%f, %f,%f \n',xstate_EKF(i,1),xstate_EKF(i,2),xstate_EKF(i,3),xstate_EKF(i,4));
end
fclose(xstate_file);


dd_file= fopen(fullfile(DIR, 'DD_98279.txt'),'wt');
for i=1:length(dd)
    fprintf(dd_file,'%f ,%f\n',dd(1,i),dd(2,i));
end
fclose(dd_file);

tri_file= fopen(fullfile(DIR, 'TRI_98279.txt'),'wt');
for i =1:length(tri)
    fprintf(tri_file,'%f ,%f\n',tri(1,i),tri(2,i));
end
fclose(tri_file);



end


