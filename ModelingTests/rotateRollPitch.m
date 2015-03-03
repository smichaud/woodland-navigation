function rotatedMatrix = rotateRollPitch( matrix, roll, pitch, unit )
% Rotate in roll then in pitch (order matter)

if strcmp(unit, 'deg')
    roll = degtorad(roll);
    pitch = degtorad(pitch);
end

rollMatrix = [1 0          0 ;
              0 cos(roll)  sin(roll) ;
              0 -sin(roll) cos(roll)];
pitchMatrix = [cos(pitch)  0 sin(pitch) ; 
               0           1 0 ;
               -sin(pitch) 0 cos(pitch)];

rotatedMatrix = (matrix'*rollMatrix*pitchMatrix)';
           
end

