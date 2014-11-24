function interpolatedVariance = interpolateVariance( x, y,...
    start, step, stop)

timeVector = (start:step:stop)';
interpolatedVariance = var(interp1(x, y, timeVector));

end

