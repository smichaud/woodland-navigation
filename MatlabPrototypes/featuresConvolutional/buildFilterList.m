if mod(filterSide,2) ~= 1 || filterSide < 3
    error('The voxel filter must be odd and greater or equal to 3')
end
filterPadding = round((filterSide - 1)/2);

nbOfSamples = length(dataset);
nbOfFilterPerSample = 0;
filters = zeros(1);