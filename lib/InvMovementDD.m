function [hr] = InvMovementDD(P,R,St,sscale)
% input= P - matriz com as coordenadas do trajecto
%         St - numero de passos
%         sscale - escala do robo
%         R - matriz com as coordenadas do robo

if nargin<3
    St=0.1; 
end
if nargin<4
 sscale=0.1;

end



Rh=R*sscale; Rh(3,:)=1 ; %homogenous verison

hr=fill(Rh(1,:),Rh(2,:),'y');
axis equal; grid on; hold on
% mesmo processo de movimento do traject, mas este acaba por ser passo a
% passo

d=diff(P,1,2);
ang=atan2(d(2,:),d(1,:)); % calculo do angulo

for n=1:St
    disp(n)
    T = transl(P(:,n))*rotat(ang(n));
    nR=T*Rh;
    hr.XData=nR(1,:);hr.YData=nR(2,:);
    pause(0.1)
end

