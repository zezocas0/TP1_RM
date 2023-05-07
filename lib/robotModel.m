function P = robotModel(t,k)

if nargin<2
    k=0.1;
end

switch t
    case 1 %DD
        P=k*[-1.5 1.5  3 1.5 -1.5 -1.5
            -1.5 -1.5 0  1.5 1.5 -1.5];

    case 2 %Triangular
        P=k*[-1.5 3 -1.5 -1.5 
            -1.5 0 1.5 -1.5];
    case 3 %OMNI
        ang=linspace(0,2*pi,7);
        P=k*[cos(ang) 
            sin(ang)];


end


end

