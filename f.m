function x_pred = f(x, u, Dt, model)
% State transition function for a simple kinematic motion model
% x: state vector [x,y,theta] (position)
% u: input vector [acc] (acceleration)
% Dt: time step
% model: motion model parameter (velocity damping factor)

A = [1 Dt; 0 1] % State transition matrix
B = [Dt^2/2; Dt] % Input matrix
G = [0; -Dt*model] % Process noise matrix

x_pred = A*x + B*u + G*randn; % State transition function with process noise

end
