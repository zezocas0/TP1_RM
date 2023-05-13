function [xstate_t1,P_t1] = ekf(xstate_t,P_t,control_t,obs_t1,landmarkxym,Delta_T,Q,R)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            funciton ekf
%   Exetended Kalman Filter
%   Modified from EKF.m  by: José Santos 98279
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%number of landmarks
N= size(landmarkxym(:,1),1);

%% prediction step
%motion model
pzero =[0 0];        %set noies equal to 0

%motion model estimate
[xstatet1_t] = motionmodel(xstate_t,control_t,pzero,Delta_T);

%predicted covariance matrix (uncertainty)
temp = -Delta_T*control_t(1)*sin(xstate_t(3));
temp2= -Delta_T*control_t(1)*cos(xstate_t(3));
Jfx=[1 0 temp
     0 1 temp2
     0 0 1     
     ];

temp3 =  Delta_T*control_t(1)*cos(xstate_t(3));
temp4 =  Delta_T*control_t(1)*sin(xstate_t(3));

Jfw=[temp3 0 
     temp4 0
     0     Delta_T
     ];

Pt1_t= Jfx*P_t*Jfx'+Jfw*Q*Jfw';     %uncertainty

%% update step

% concatenate all observations and predicted observations
z_all = [];
z_pred = [];
Jh = [];

obs_i=obs_t1;
for i = 1:N % modified part(for N beacons besides 2)
    
    obs_i = obs_t1;
    
    num_triplets = length(obs_i) / 3;
    idx = 3*(i-1) + 1;
    % Extract the id value
    id = obs_i(idx);
    
    %xy coords of the lanrmark with number id
    landmark_i= landmarkxym(id,2:3);
    
    
    nzero = [0 0]; %set noise equal to 0
    
    
    % get the observation and predicted observation
    z_i = [ obs_i(idx+1) obs_i(idx+2)];
    
    z_pred_i = sensormodel(landmark_i,xstatet1_t,nzero);
    
    % add to the concatenated vectors
    z_all = [z_all; z_i'];
    z_pred = [z_pred; z_pred_i'];
    
    % compute the Jacobian and add to the concatenated matrix
    Jh_i = jacobi(landmark_i,xstatet1_t(1),xstatet1_t(2));
    Jh = [Jh; Jh_i];
    



end

% innovation
innov = z_all - z_pred;
% wrap the angles to [-pi, pi]
innov(2:2:end)=wrap(innov(2:2:end));

%%%% To remove NaN points from beaconDetection from obs_range_bearing 
innov(isnan(innov))=1; % if landmark is not observed, set innovation to 0
%%%%


S = Jh*Pt1_t*Jh'+R;
K = Pt1_t*Jh'*inv(S);

%result
xstatet1_t1 = xstatet1_t'+K*innov;
Pt1_t1      = Pt1_t - K*Jh*Pt1_t;

xstate_t1 = xstatet1_t1';
P_t1      = Pt1_t1;


end



function nu = wrap(alpha)

clear nu;
nu = alpha;

	while (nu > pi)
		nu = nu - 2 * pi;
    end

	while (nu < -pi)
		nu = nu + 2 * pi;
    end
end