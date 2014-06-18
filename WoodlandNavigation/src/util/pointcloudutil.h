#ifndef POINTCLOUDUTIL_H
#define POINTCLOUDUTIL_H

#include "definitions.h"

class PointCloudUtil
{
public:
    static void appendDescriptorRGBA(PM::DataPoints &dataPoints, float red,
                                     float green, float blue, float alpha);
    static PM::DataPoints getSinglePointPerVoxel();
};

#endif
