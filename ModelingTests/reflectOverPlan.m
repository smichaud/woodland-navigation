function reflectedMatrix = reflectOverPlan( matrix, a, b, c )
% Reflect the matrix over a plane defined by the vector [a,b,c]

planeNormal = [a b c];
planeNormal = planeNormal/norm(planeNormal);

reflectionMatrix = eye(3)-2*planeNormal*planeNormal';
      
reflectedMatrix = (matrix'*reflectionMatrix)';

end