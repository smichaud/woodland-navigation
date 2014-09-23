% Get the mean position in x, y and z (center of mass of points)
nbOfSamples = length(dataset);
for i=1:nbOfSamples    
%     dataset(i).features('meanX') = mean(dataset(i).areaOfInterest(:,1));
%     dataset(i).features('meanY') = mean(dataset(i).areaOfInterest(:,2));
    dataset(i).features('meanZ') = mean(dataset(i).areaOfInterest(:,3));
end