function [dataset completeWordList] = buildWordLists(...
    dataset, wordGridSize)

if mod(wordGridSize,2) ~= 1 || wordGridSize < 3
    error('The wordGridSize must be odd and greater or equal to 3')
end
padVoxelsCount = round((wordGridSize - 1)/2);

samplesCount = length(dataset);
wordsCountPerSample = (nbOfX-2*nbOfPaddingVoxel)*...
    (nbOfY-2*nbOfPaddingVoxel)*(nbOfZ-2*nbOfPaddingVoxel);

completeWordList = zeros(samplesCount*wordsCountPerSample, );

overallIndex = 1;
for sampleIndex = 1:samplesCount
    sampleWordIndex = 1;
    dataset(sampleIndex).words = ...
        zeros(wordsCountPerSample, wordGridSize^3);
    
    for xIndex = 1+padVoxelsCount:nbOfX-padVoxelsCount
        for yIndex = 1+padVoxelsCount:nbOfY-padVoxelsCount
            for zIndex = 1+padVoxelsCount:nbOfZ-padVoxelsCount
                rawRatioGrid = dataset(sampleIndex).pointsCountVoxelGrid(...
                    xIndex-padVoxelsCount:xIndex+padVoxelsCount,...
                    yIndex-padVoxelsCount:yIndex+padVoxelsCount,...
                    zIndex-padVoxelsCount:zIndex+padVoxelsCount);
                
                if sum(rawRatioGrid(:)) == 0
                    completeWordList(overallIndex,:) = 0;
                else
                    completeWordList(overallIndex,:) = ...
                        reshape(rawRatioGrid,1,[])./sum(rawRatioGrid(:));                    
                end
                
                dataset(sampleIndex).ratioGridVectors(sampleWordIndex,:) = ...
                    completeWordList(overallIndex,:);
                
                sampleWordIndex = sampleWordIndex + 1;
                overallIndex = overallIndex + 1;
            end
        end
    end
end

end

