tic

filterIndex = 1;
for sampleIndex = 1:nbOfSamples
    for xIndex = 1+nbOfPaddingVoxel:nbOfX-nbOfPaddingVoxel
        for yIndex = 1+nbOfPaddingVoxel:nbOfY-nbOfPaddingVoxel
            for zIndex = 1+nbOfPaddingVoxel:nbOfZ-nbOfPaddingVoxel
                rawFilter = dataset(sampleIndex).voxelMap(...
                    xIndex-nbOfPaddingVoxel:xIndex+nbOfPaddingVoxel,...
                    yIndex-nbOfPaddingVoxel:yIndex+nbOfPaddingVoxel,...
                    zIndex-nbOfPaddingVoxel:zIndex+nbOfPaddingVoxel);
                
                if sum(rawFilter(:)) == 0
                    filters(filterIndex,:) = 0;
                else
                    filters(filterIndex,:) = ...
                        reshape(rawFilter,1,[])./sum(rawFilter(:));
                    
                end
                
                filterIndex = filterIndex + 1;
            end
        end
    end
end

createFilterListTime = toc