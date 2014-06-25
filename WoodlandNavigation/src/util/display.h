#ifndef DISPLAY_H
#define DISPLAY_H

#include "pointcloud.h"
#include "definitions.h"

class Display
{
public:
    static void printAllInfo(const PointCloud &pointcloud);

    static void printNumberOfPoints(const PointCloud &pointcloud);

    static void printFeatures(const PointCloud &pointcloud);
    static void printDescriptors(const PointCloud &pointcloud);
};

#endif
