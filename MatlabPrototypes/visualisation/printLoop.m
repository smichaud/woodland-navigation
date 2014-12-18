nbOfSamples = length(dataset);
good = 0;
bad = 0;
for i=1:nbOfSamples
    if dataset(i).rawCurrents(1,2) > 8
        bad = bad+1;
    else
        good = good+1;
    end
    nbOfPoints(i) = size(dataset(i).areaOfInterest,1);
%     disp(sprintf('%s : %f === %f',...
%         dataset(i).name, ...
%         dataset(i).features('density'), ...
%         dataset(i).features('highestPoint')));
end

% minNbOfPoints = min(nbOfPoints)
% maxNbOfPoints = max(nbOfPoints)
% meanNbOfPoints = mean(nbOfPoints)
% medianNbOfPoints = median(nbOfPoints)

