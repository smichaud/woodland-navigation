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
                        const Vector3 &voxelSize = Vector3::Zero());

    Vector3 getPointsBoundingBoxMin(const PM::Matrix &features);
    Vector3 getPointsBoundingBoxMax(const PM::Matrix &features);
    Vector3 getNbVoxel();

private:
    void computeGridMinMaxCorners(const PM::Matrix &features);

    // I am not sure it is useful to have to voxel class right now...
//    void setVoxelsCorners(Vector3 voxelSize, Vector3 nbVoxel);
    void buildVoxelGridStructure(const PM::DataPoints &dataPoints,
                                 const Vector3 &nbVoxel);
    Vector3 getVoxelIndice(Vector3 pointsPosition);
};


#endif
