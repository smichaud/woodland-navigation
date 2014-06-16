#ifndef VOXELGRIDPOINTCLOUD_H
#define VOXELGRIDPOINTCLOUD_H

#include "voxel.h"
#include "definitions.h"

class VoxelGridPointCloud
{
private:
    Vector3 voxelSize;

    Vector3 minCorner;
    Vector3 maxCorner;

    PM::DataPoints completeDataPoints;
    std::vector<std::vector<std::vector<Voxel> > > voxels;

public:
    VoxelGridPointCloud(const PM::DataPoints &dataPoints, Vector3 voxelSize);

    PM::DataPoints getSinglePointPerVoxel();

private:
    void getPointsBoundingBoxMin(const PM::Matrix &pointCloud);
    void getPointsBoundingBoxMax(const PM::Matrix &pointCloud);
};


#endif
