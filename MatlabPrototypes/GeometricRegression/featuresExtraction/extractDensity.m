volume = areaOfInterest.width * areaOfInterest.height ...
    * areaOfInterest.depth;

nbOfSamples = length(dataset);
for i=1:nbOfSamples
    nbOfPoints = size(dataset(i).areaOfInterest,1);
    dataset(i).features('density') = nbOfPoints/volume;
end
