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
    VoxelGridPointCloud();
    VoxelGridPointCloud(const PM::DataPoints &dataPoints,
                        used_type voxelSizeX=0, used_type voxelSizeY=0,
                        used_type voxelSizeZ=0);
    VoxelGridPointCloud(const PM::DataPoints &dataPoints,
                        Vector3 voxelSize = Vector3::Zero());

    PM::DataPoints getSinglePointPerVoxel();

private:
    Vector3 getPointsBoundingBoxMin(const PM::Matrix &features);
    Vector3 getPointsBoundingBoxMax(const PM::Matrix &features);

    void getGridMinCorner(const Vector3 &voxelSize,
                          const PM::Matrix &features);
    void getGridMaxCorner(const Vector3 &voxelSize,
                          const PM::Matrix &features);

    void setVoxelsCorners(Vector3 voxelSize, Vector3 nbVoxel);
    void buildVoxelGridStructure(Vector3 voxelSize);
};


#endif
