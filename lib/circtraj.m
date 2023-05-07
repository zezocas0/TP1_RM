function MM = circtraj(P1,P2,R,N)
% MM.xy - matrix of xy points
% MM.angle - vector of orientations
% MM.T - associate geometric transformation

A = P1; %Point A to be on circumference
B = P2; %Same with point B
d = norm(B-A); 
R = max(R,d/2); %ensure R radius >= d/2

if isinf(R) 
    R = 1e6;    %use a very large R
end

C = (B+A)/2+sqrt(R^2-d^2/4)/d*[0,-1;1,0]*(B-A);  %One Center
% C = (B+A)/2-sqrt(R^2-d^2/4)/d*[0,-1;1,0]*(B-A); %Other Center
a = atan2(A(2)-C(2), A(1)-C(1)); %Angle
b = atan2(B(2)-C(2), B(1)-C(1));
b = mod(b-a,2*pi)+a; % Ensure that arc goes counterclockwise
t = linspace(a,b,N);
M = C+R*[cos(t); sin(t)];

dM = diff(M,1,2);   %first difference along dimension 2
ang = atan2(dM(2,:), dM(1,:));
ang(end+1) = ang(end);

T = zeros(3,3,size(M,2));
for n = 1:size(T,3)
    T(:,:,n) = trans1(M(:,n))*rotat(ang(n));
end

MM.xy=M;
MM.angle=ang;
MM.T=T;









end

