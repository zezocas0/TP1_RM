function [F, G] = jacobian_matrices(x, u, Dt, model)
% Calculates the Jacobian matrices for the state transition function
% INPUTS:
% x - current state estimate (x,y,theta)
% u - control input (linear velocity, angular velocity)
% Dt - time step
% model - robot model selection (1: differential drive, 2: tricycle)
% OUTPUTS:
% F - Jacobian matrix of the state transition function
% G - Jacobian matrix of the process noise

% Initialize Jacobian matrices
F = eye(3);
G = zeros(3, 2);

% Calculate the Jacobian matrices for the differential drive model
if model == 1
    theta = x(3);
    v = u(1);
    F(1, 3) = -v * sin(theta) * Dt;
    F(2, 3) = v * cos(theta) * Dt;
    G(1, 1) = cos(theta) * Dt;
    G(2, 1) = sin(theta) * Dt;
    G(3, 2) = Dt;
end

% Calculate the Jacobian matrices for the tricycle model
if model == 2
    theta = x(3);
    v = u(1);
    phi = u(2);
    F(1, 3) = -v / phi * sin(theta) + v / phi * sin(theta + phi * Dt);
    F(2, 3) = v / phi * cos(theta) - v / phi * cos(theta + phi * Dt);
    G(1, 1) = -1 / phi * sin(theta) + 1 / phi * sin(theta + phi * Dt);
    G(1, 2) = v / phi^2 * (sin(theta) - sin(theta + phi * Dt)) + v / phi^2 * cos(theta + phi * Dt) * Dt;
    G(2, 1) = 1 / phi * cos(theta) - 1 / phi * cos(theta + phi * Dt);
    G(2, 2) = -v / phi^2 * (cos(theta) - cos(theta + phi * Dt)) + v / phi^2 * sin(theta + phi * Dt) * Dt;
    G(3, 2) = Dt;
end




end