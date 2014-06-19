#ifndef VOXELGRIDPOINTCLOUD_H
#define VOXELGRIDPOINTCLOUD_H

#include "voxel.h"
#include "definitions.h"


// 1. The current center of the voxel grid is the center of the point cloud
// bounding box.
// 2 . The Voxel structure only contains the indices of the points contained
// in the voxel and some general info/stats about this voxel
class VoxelGridPointCloud
{
private:
    PM::DataPoints completeDataPoints;

    Vector3 pointCloudBoundingBoxMin;
    Vector3 pointCloudBoundingBoxMax;

    Vector3 voxelSize;
    Vector3uli nbOfVoxels;
    Vector3 minVoxelLowerCorner;
    Vector3 maxVoxelLowerCorner;

    std::vector<std::vector<std::vector<Voxel> > > voxels;

public:
    VoxelGridPointCloud();
    VoxelGridPointCloud(const PM::DataPoints &dataPoints,
                        used_type voxelSizeX=0, used_type voxelSizeY=0,
                        used_type voxelSizeZ=0);
    VoxelGridPointCloud(const PM::DataPoints &dataPoints,
                        const Vector3 &voxelSize = Vector3::Zero());

    Vector3 getVoxelSize() const;
//    Vector3uli getNbVoxels() const;

    Vector3uli getNbOfVoxels() const;

private:
    void buildVoxelGridPointCloud(const Vector3 &voxelSize);
    void computePointCloudBoundingBox(const PM::Matrix &features);
    void computeNbOfVoxels();
    void computeGridMinMaxCorners();

    void buildVoxelGridStructure(const PM::DataPoints &dataPoints,
                                 const Vector3 &nbVoxel);

    Vector3 getVoxelIndice(Vector3 pointsPosition);
};


#endif
