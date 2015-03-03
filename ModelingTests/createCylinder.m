function cylinder = createCylinder( radius, minZ, maxZ, nbPoints )

switch nargin 
    case 0
        error('Arg plz');
    case 1
        minZ = -1;
        maxZ = 1;
    case 2
        maxZ = minZ + 2;
    case 3
        nbPoints = 100;
end

if radius <= 0
    error('radius <= 0');
elseif nbPoints <= 0
    error('nbPoints <= 0')
end 

x = (2*radius).*rand(1, nbPoints) - radius;
y = sqrt(radius^2-x.^2); % Only positive for now
y(2:2:end) = -y(2:2:end);
z = (maxZ-minZ).*rand(1, nbPoints) + minZ; % Random between min-max

cylinder = [x;y;z];

end

