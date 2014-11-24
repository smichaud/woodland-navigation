function interpolatedIntegral = interpolateIntegral( x, y,...
    start, step, stop)

timeVector = (start:step:stop)';
interpolatedIntegral = sum(interp1(x, y, timeVector))*step;

end
