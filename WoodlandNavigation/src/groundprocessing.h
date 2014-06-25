#ifndef GROUNDPROCESSING_H
#define GROUNDPROCESSING_H

#include "voxelgridpointcloud.h"

class GroundProcessing
{
public:
    //TODO I assume there is one voxel in height
    static void addGroundDescriptor(VoxelGridPointCloud &scene);
};

#endif
