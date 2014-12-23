% Get the mean position in x, y and z (center of mass of points)
nbOfSamples = length(dataset);
for i=1:nbOfSamples
    if ~isempty(dataset(i).areaOfInterest)
        dataset(i).features('meanX') = mean(dataset(i).areaOfInterest(:,1));
        dataset(i).features('meanY') = mean(dataset(i).areaOfInterest(:,2));
        dataset(i).features('meanZ') = mean(dataset(i).areaOfInterest(:,3));
    else
        warning(['areaOfInterest contains no point, '...
            'mean feature will be set to 0, 0, 0']);
        dataset(i).features('meanX') = 0;
        dataset(i).features('meanY') = 0;
        dataset(i).features('meanZ') = 0;
    end
end