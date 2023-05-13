function [xstate_true,control_input_true,control_input_mea,obs_range_bearing,landmarkxy,obs_landmark_ID]=generate_ekf_data(N,sig_v,sig_omega,sig_r,sig_phi,Dt,vels,theta,pchip_data)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            funciton generate_ekf_data
%   Function to create the inputs for the EKF functions(to be able to use N beacons)
%   Modified from Generate_data_EKF_localization_V3.m by: José Santos 98279
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



print("generaging data")

v=vels(1,:);
w=vels(2,:);

xinterp=pchip_data(1,:);
yinterp=pchip_data(2,:);

B=BeaconDetection(N,[0,0,0]);

landmarkxy=[];
for i=1:N
    landmarkxy= [landmarkxy;i, B(i).X,B(i).Y];
end




%% control inputs, format: time_step, linear velocity, turnrate (angular velocity)
control_input_true=[];
for i=1:length(v)
    control_input_true = [control_input_true;i, v(i) ,w(i)];
end

% number of motion steps (starts from time step 0)
num_steps = height(control_input_true);

%generating measured control inputs by adding noises
control_input_mea     =control_input_true;
%control noises
noises_v              =randn(num_steps,1)*sig_v;
noises_omega          =randn(num_steps,1)*sig_omega;
control_input_mea(:,2)=control_input_mea(:,2)+noises_v;
control_input_mea(:,3)=control_input_mea(:,3)+noises_omega;





%% generate ground true robot poses(true state for EKF)
% format: pose ID, x, y, phi
xstate_true = [0, xinterp(1), yinterp(1), 0]; % pose at time 0
for i=1:num_steps
    control_i = control_input_true(i,2:3);
    control_noise = [0;0];
    %xinterp, yinterp and theta of angles, motion model is not needed Due to xinterp,yinterp and theta are already the true values
    xstate_true=[xstate_true; i xinterp(i) yinterp(i) theta(i)];    
end

%% generating observation data
obs_landmark_ID = [];
for i=1:length(xinterp)-1
    % choose randomly the organization of N landmarks
    landmark_ID = randperm(N,N);
    obs_landmark_ID = [obs_landmark_ID;i landmark_ID];
end


%Format: time_step,landmarkID1,bearing1,range1,landmarkID2,bearing2,range2,...
obs_range_bearing = [];
for i=1:num_steps    
    for j=1:N
        id(j)=obs_landmark_ID(i,j+1);
    end 
    % observation noises
    noise_r      =randn*sig_r;
    noise_phi    =randn*sig_phi;
    sensor_noise =[noise_r  noise_phi];
    %BeaconDetection for each step
    B_bearing=BeaconDetection(N,xstate_true(i+1,2:4)); 
    z=[];
    %z=based on variable id, get the range and bearing of landmark with that id. z is a vector with id,range,bearing

    for j = 1:N
        z=[z,id(j) B_bearing(id(j)).d B_bearing(id(j)).a];
    end

    obs_range_bearing = [obs_range_bearing;
                         i z ];
    
end

