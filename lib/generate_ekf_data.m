function [xstate_true,control_input_true,control_input_mea,obs_range_bearing,landmarkxy,obs_landmark_ID]=generate_ekf_data(N,sig_v,sig_omega,sig_r,sig_phi,Dt,vels,theta,pchip_data)
%INPUTS
%N: number of landmarks
%sig_v: standard deviation of linear velocity noise
%sig_omega: standard deviation of angular velocity noise
%sig_r: standard deviation of range noise
%sig_phi: standard deviation of bearing noise
%Dt: sampling time
%vels: velocity data[v- linear velocity,w- angular velocity]
%theta: heading data
%pchip_data: interpolated data[xinterp,yinterp]
%OUTPUTS
%xstate_true: ground truth of robot poses
%control_input_true: true control inputs
%control_input_mea: measured control inputs
%obs_range_bearing: range and bearing observations
%landmarkxy: landmark positions




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





%% generate ground true robot poses
% format: pose ID, x, y, phi
xstate_true = [0, xinterp(1), yinterp(1), 0]; % pose at time 0
for i=1:num_steps
    control_i = control_input_true(i,2:3);
    control_noise = [0;0];
    %sampling time
    %xinterp, yinterp and theta of angles, motion model is not needed Due to xinterp,yinterp and theta are already the true valuesm, no other action needed
    % xstatet1 = motionmodel([xinterp(i),yinterp(i),theta(i)],control_i,control_noise,Dt);
    xstate_true=[xstate_true; i xinterp(i) yinterp(i) theta(i)];    
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



obs_range_bearing = [];
for i=1:num_steps    
    id1=obs_landmark_ID(i,2);
    id2=obs_landmark_ID(i,3);
    % observation noises
    noise_r      =randn*sig_r;
    noise_phi    =randn*sig_phi;
    sensor_noise =[noise_r  noise_phi];
    
    B_bearing=BeaconDetection(N,xstate_true(i+1,2:4)); 

    % range-bearing to one landmark
    z1 =[B_bearing(id1).d B_bearing(id1).a] +sensor_noise;
    % range-bearing to another landmark
    z2=[B_bearing(id2).d B_bearing(id2).a]+sensor_noise;


    % check if any value in z1 is NaN
    if any(isnan(z1))
        % randomize a new ID and check if it's NaN
        new_id = randi(N);
        while any(isnan([B_bearing(new_id).d B_bearing(new_id).a]))
            new_id = randi(N);
        end
        % replace NaN value with data related to new_id
        if isnan(z1(1))
            z1 = [B_bearing(new_id).d B_bearing(new_id).a] + sensor_noise;
            id1 = new_id;
        else
            z1(2) = B_bearing(new_id).a + noise_phi;
            id1 = new_id;
        end
    end

    % check if any value in z2 is NaN
    if any(isnan(z2))
        % randomize a new ID and check if it's NaN
        new_id = randi(N);
        while any(isnan([B_bearing(new_id).d B_bearing(new_id).a]))
            new_id = randi(N);
        end
        % replace NaN value with data related to new_id
        if isnan(z2(1))
            z2 = [B_bearing(new_id).d B_bearing(new_id).a] + sensor_noise;
            id2 = new_id;
        else
            z2(2) = B_bearing(new_id).a + noise_phi;
            id2 = new_id;
        end
    end
    


    % store the obs data
    obs_range_bearing = [obs_range_bearing;
                         i id1 z1 id2 z2];
    

end
