#ifndef UTIL_H
#define UTIL_H

#include "pointmatcher/PointMatcher.h"

typedef PointMatcher<float> PM;

class Util
{
public:
    static void appendDescriptorRGBA(PM::DataPoints &dataPoints, float red,
                                     float green, float blue, float alpha);
};

#endif
