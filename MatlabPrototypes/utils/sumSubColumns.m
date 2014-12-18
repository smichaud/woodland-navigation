function [ sumData ] = sumSubColumns( data, nbOfColumnsStep )

sumData = zeros(size(data,1), floor(size(data,2)/nbOfColumnsStep));
for i = 1:nbOfColumnsStep:size(data,2)
    lastIndex = i+nbOfColumnsStep-1;
    if lastIndex > size(data,2)
        lastIndex = size(data,2);
    end
    
    sumData(:,((i-1)/nbOfColumnsStep)+1) = ...
        sum(data(:, i:lastIndex),2);
end

end

