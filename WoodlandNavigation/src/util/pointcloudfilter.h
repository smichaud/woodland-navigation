#ifndef POINTCLOUDFILTER_H
#define POINTCLOUDFILTER_H

#include "pointcloud.h"

namespace PointCloudFilter {

void minRadius(PointCloud &pointcloud, float radius);
void maxRadius(PointCloud &pointCloud, float radius);
void maxDensity(PointCloud &pointCloud, float density);
void boundingBox(PointCloud &pointCloud,
                 float xMin, float xMax,
                 float yMin, float yMax,
                 float zMin, float zMax,
                 bool removeInside = 0);

}

#endif
