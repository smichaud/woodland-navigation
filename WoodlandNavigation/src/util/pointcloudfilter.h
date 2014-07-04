#ifndef POINTCLOUDFILTER_H
#define POINTCLOUDFILTER_H

#include "pointcloud.h"

class PointCloudFilter
{
public:
    static void minRadius(PointCloud &pointcloud, float radius);
    static void maxRadius(PointCloud &pointCloud, float radius);
    static void maxDensity(PointCloud &pointCloud, float density);
    static void boundingBox(PointCloud &pointCloud,
                            float xMin, float xMax,
                            float yMin, float yMax,
                            float zMin, float zMax,
                            bool removeInside = 0);
};

#endif
