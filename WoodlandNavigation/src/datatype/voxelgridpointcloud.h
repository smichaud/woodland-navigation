/* Notes:
 * 1. The current center of the voxel grid is the center of the point cloud
 *    bounding box.
 * 2.If the voxelSize is <= 0 in a certain orientation, the voxel will be the
 *   size of the pointcloud boundingbox in this direction
*/

#ifndef VOXELGRIDPOINTCLOUD_H
#define VOXELGRIDPOINTCLOUD_H

#include "pointcloud.h"
#include "voxel.h"
#include "definitions.h"


class VoxelGridPointCloud
{
private:
    PointCloud pointCloud;

    Vector3 pointCloudBoundingBoxMin;
    Vector3 pointCloudBoundingBoxMax;

    Vector3 voxelSize;
    Vector3uli nbOfVoxels;
    Vector3 minVoxelLowerCorner;
    Vector3 maxVoxelLowerCorner;

    std::vector<std::vector<std::vector<Voxel> > > voxels;

    static const used_type minPadding;

public:
    VoxelGridPointCloud();
    VoxelGridPointCloud(PointCloud &pointCloud,
                        used_type voxelSizeX=0, used_type voxelSizeY=0,
                        used_type voxelSizeZ=0);
    VoxelGridPointCloud(PointCloud &pointCloud,
                        const Vector3 &voxelSize = Vector3::Zero());

    void addDescriptor(const std::string name, VectorX descriptorDefaultValue);

    PointCloud &getPointCloudRef();
    Vector3 getVoxelSize() const;
    Vector3uli getNbVoxels() const;
    Voxel getVoxel(uli x, uli y, uli z);


private:
    void buildVoxelGridPointCloud();
    void computePointCloudBoundingBox();
    void verifyVoxelSize();
    void computeNbOfVoxels();
    void computeGridMinMaxCorners();

    void initVoxels();
    void buildVoxels();

    Vector3uli getVoxelIndex(Vector3 pointPosition);
};


#endif
