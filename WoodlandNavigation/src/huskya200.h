#ifndef HUSKYA200_H
#define HUSKYA200_H

#include "groundvehicle.h"
#include "definitions.h"
#include "voxelgridpointcloud.h"
#include "pointcloud.h"
#include "voxel.h"

class HuskyA200 : public GroundVehicle
{
private:
    const used_type groundMaxAngleInRad;
    const used_type minEffectiveSensorRange;
    const used_type maxEffectiveSensorRange;

    const unsigned int knnUsedForPointCloud;
    const used_type maxPointCloudDensity;

    //TODO change to some kind of float enum / class
    const std::string navigationLabel;
    const used_type label_unknown;
    const used_type label_ground;
    const used_type label_tree;

    PointCloud currentPointCloud;

public:
    HuskyA200();

public:
    void gotPointCloud(PointCloud pointCloud);
private:
    void preprocessPointCloud();

    void segmentGroud();
    VoxelGridPointCloud createColumnVoxels(used_type colVoxelBaseSideSize);
    void segmentVoxelGround(const Voxel &voxel);
    uli guessVoxelGroundHeight(const Voxel &voxel) const;
    uli getVoxelQuantileIndex(const Voxel &voxel, const float quantile) const;
    bool isPossibleGroundAngle(const uli pointIndex) const;
    bool isPlanarRegion(const uli pointIndex) const;
    void labelAllPointsInThresholdAsGround(const Voxel &voxel,
                                           const uli groundPointIndex,
                                           const used_type threshold);
    void unlabelIfNotEnoughPoints(
            const std::vector<uli> &groundPointsInThreshold);

    void segmentTrees();

public:
    void savePointCloudRepresentation(const std::string filename) const;
};

#endif
