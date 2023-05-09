function [cv] = GetCov(P,x,y)
%GETCOV Summary of this function goes here
%   Detailed explanation goes here
s=1;
k=40;

if trace(P)<1e-5
    r=zeros(2,2);
else
    r=real(sqrt(P));
end

for j=1:k+1
    q=2*pi*(j-1)/k;
    cv(:,j)=s*3*r*[cos(q);sin(q)]+[x;y];
end

