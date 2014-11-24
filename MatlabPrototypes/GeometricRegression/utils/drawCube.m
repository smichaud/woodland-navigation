function drawCube( center, side, color )
% Draw a cube in the figure

if nargin < 2
    error('You must give a the center and the side length as input');
end
if nargin < 3
    color = 'r';
end

vertice = [1 1 -1; 
        -1 1 -1; 
        -1 1 1; 
        1 1 1; 
        -1 -1 1;
        1 -1 1; 
        1 -1 -1;
        -1 -1 -1];
vertice = vertice.*(side/2);
vertice = vertice + repmat([center(1), center(2), center(3)], 8, 1);

faces = [1 2 3 4; 
       4 3 5 6; 
       6 7 8 5; 
       1 2 8 7; 
       6 7 1 4; 
       2 3 5 8];

patch('Faces',faces,'Vertices',vertice,'FaceColor',color);

material metal;
alpha('color');
alphamap('rampdown');

end

