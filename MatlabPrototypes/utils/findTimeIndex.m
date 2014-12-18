function index = findTimeIndex( desiredTime, timeVector )
% Return the index of the value(time) closest to the desired value(time)

difference = abs(timeVector - desiredTime);
indexes = find(min(difference) == difference);

index = indexes(1);

end

