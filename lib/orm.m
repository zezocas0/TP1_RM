function [R] = orm(theta)
%ORM returns the orthogonal rotational matrix for angle theta(rad)

%   Detailed explanation goes here

R=[cos(theta) sin(theta) 0 
    -sin(theta) cos(theta) 0
    0 0 1];

end

