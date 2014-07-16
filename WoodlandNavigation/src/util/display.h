#ifndef DISPLAY_H
#define DISPLAY_H

#include "pointcloud.h"
#include "definitions.h"

namespace Display {

void printAllInfo(const PointCloud &pointcloud);

void printNumberOfPoints(const PointCloud &pointcloud);

void printFeatures(const PointCloud &pointcloud);
void printDescriptors(const PointCloud &pointcloud);

}

#endif
