nbOfFilterPerSample = (nbOfX-2*nbOfPaddingVoxel)*...
    (nbOfY-2*nbOfPaddingVoxel)*(nbOfZ-2*nbOfPaddingVoxel);
totalNbOfFilter = nbOfFilterPerSample*nbOfSamples;

filters = zeros(totalNbOfFilter, filterSide^3);

filterIndex = 1;
for sampleIndex = 1:1 %nbOfSamples
%     startIndex = (sampleIndex-1)*nbOfFilterPerSample + 1;
%     endIndex = startIndex + nbOfFilterPerSample - 1;
    for xIndex = 1+nbOfPaddingVoxel:nbOfX-2*nbOfPaddingVoxel
        for yIndex = 1+nbOfPaddingVoxel:nbOfY-2*nbOfPaddingVoxel
            for zIndex = 1+nbOfPaddingVoxel:nbOfZ-2*nbOfPaddingVoxel
                rawFilter = dataset(sampleIndex);
                filters(filterIndex,:) = ...
                    reshape(rawFilter,1,[])./sum(rawFilter(:));
                
                filterIndex = filterIndex + 1;
            end
        end
    end
    
%     filters(startIndex:endIndex,:) = 
end