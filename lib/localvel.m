function [Vx,Vy,w] = localvel(t,r,L,w1,aw2,w3)
%lOCALVEL Summary of this function goes here

% Vx,Vy,w- velocities in local frame
% t- tye of fobot
%     1- DD   
%     2-Tri
%     3- Omni
% r- traction wheel radius
% L- Different meaning (wheel sep/1 , wheel dist/2), wheel dist/3)
% w1- angular vel of wheel 1 (right wheel)
% aw2- angular vel of wheel 2(left/1) or alpha/2
% w3- angular vel of wheel 2 (Omni)

if nargin<6
    w3=0;
end


switch t
    case 1
        m=[r/2 r/2
            0 0 
            -r/L r/L];
        V=m*[aw2 ; w1];
        Vx=V(1);
        Vy=V(2);
        w=V(3);
    case 2
        Vx=w1*r*cos(aw2);
        Vy=0;
        w=w1*r/L*sin(aw2);

    case 3
        m=[0 r/sqrt(3) -r/sqrt(3)
            -2*r/sqrt(3) r/3 r/3
            r/(3*L) r/(3*L) r/(3*L)
            ];
        v=m*[w1;aw2;w3];
        Vx=v(1);
        Vy=v(2);
        w=v(3);
end

