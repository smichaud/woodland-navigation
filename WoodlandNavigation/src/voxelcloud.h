#ifndef VOXELCLOUD_H
#define VOXELCLOUD_H

#include "pointmatcher/PointMatcher.h"
#include "voxel.h"

typedef PointMatcher<float> PM;

class VoxelCloud
{
private:
    static const int xIndice;
    static const int yIndice;
    static const int zIndice;

    float voxelSize;

    float minPointX;
    float maxPointX;
    float minPointY;
    float maxPointY;
    float minPointZ;
    float maxPointZ;

    float minCornerX;
    float maxCornerX;
    float minCornerY;
    float maxCornerY;
    float minCornerZ;
    float maxCornerZ;

    std::vector<std::vector <std::vector <Voxel> > > voxelGrid;

public:
    VoxelCloud(PM::DataPoints &dataPoints, float voxelSize);    

    PM::DataPoints getSinglePointPerVoxel();

private:
    void findPointCloudBoundingBox(PM::Matrix pointCloud);
};


#endif
