tic

ratioGridIndex = 1;
for sampleIndex = 1:nbOfSamples
    sampleRatioGridIndex = 1;
    dataset(sampleIndex).ratioGridVectors = ...
        zeros(nbOfRatioGridsPerSample, ratioGridSide^3);
    for xIndex = 1+nbOfPaddingVoxel:nbOfX-nbOfPaddingVoxel
        for yIndex = 1+nbOfPaddingVoxel:nbOfY-nbOfPaddingVoxel
            for zIndex = 1+nbOfPaddingVoxel:nbOfZ-nbOfPaddingVoxel
                rawRatioGrid = dataset(sampleIndex).voxelCardinalityMap(...
                    xIndex-nbOfPaddingVoxel:xIndex+nbOfPaddingVoxel,...
                    yIndex-nbOfPaddingVoxel:yIndex+nbOfPaddingVoxel,...
                    zIndex-nbOfPaddingVoxel:zIndex+nbOfPaddingVoxel);
                
                if sum(rawRatioGrid(:)) == 0
                    ratioGrids(ratioGridIndex,:) = 0;
                else
                    ratioGrids(ratioGridIndex,:) = ...
                        reshape(rawRatioGrid,1,[])./sum(rawRatioGrid(:));                    
                end
                
                dataset(sampleIndex).ratioGridVectors(sampleRatioGridIndex,:) = ...
                    ratioGrids(ratioGridIndex,:);
                
                sampleRatioGridIndex = sampleRatioGridIndex + 1;
                ratioGridIndex = ratioGridIndex + 1;
            end
        end
    end
end

createRatioGridsTime = toc