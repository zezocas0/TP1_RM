function [xstate_EKF,P_EKF]=ekf_calculations(landmarkxy,control_input_mea,control_input_true,obs_range_bearing,xstate_true,obs_landmark_ID,uncertainty,obs_noise,plotting)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            funciton ekf_calculations
%   Function to create the inputs for the EKF functions(to be able to use N beacons)
%   Modified from Robot_Localization_EKF_landmark_v3.m  by: José Santos 98279
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% unpacking the noises  
Vn=uncertainty(1);
Wn=uncertainty(2);
sig_r=obs_noise(1);
sig_phi=obs_noise(2);
N= size(landmarkxy(:,1),1);


% for storing the results
xstate_EKF = [0, zeros(1,3)]; % pose at time 0
P_EKF      = 0.01*eye(3);  % initial covariance matrix
% noise
Q=[Vn^2,0;0,Wn^2];

R_i=[sig_r^2,0;0 sig_phi^2];

num_steps = height(control_input_true);


fprintf("EKF running...    \n")
%%EKF calculations 
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
    R=kron(eye(N),R_i);
    
    %using ekf function
    [xstateT1_T1,PT1_T1] = ekf(xstate_t,P_t,control_t,obs_t1,landmarkxy,Delta_T,Q,R);
    
    %update
    xstate_EKF = [xstate_EKF; step, xstateT1_T1];
    P_EKF = [P_EKF; PT1_T1];
end
fprintf("\n EKF done! ")

%% comparing results. calculating error


error_state=xstate_EKF(:,2:4)-xstate_true(:,2:4);
%Adding ID
error_state=[xstate_EKF(:,1),error_state];


%% draw the estimated robot poses and uncertainty ellipses



fprintf("\n starting EKF plotting... \n ")

figure(5)
set(5, 'Position', [100,100, 800, 800])
hold on;

%beacons locations
% plot(landmarkxy(:,2),landmarkxy(:,3),'bo', 'MarkerSize', 12, 'LineWidth',2);

plot(landmarkxy(:,2),landmarkxy(:,3),'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'LineWidth',2);


arrow_length=5;
%plotting  the reason for making all this callculations!
for i=1:num_steps
    fprintf(1,'\b\b\b\b\b %.2d %%',floor(i/num_steps*100))%progress status



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

    drawnow % update the plot

end
legend('Landmarks','estimated position','est. orientation','true position','true orientation','Location','northwest')    

fprintf("\n EKF plotting done! \n\n ")

