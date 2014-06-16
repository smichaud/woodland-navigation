#ifndef UTIL_H
#define UTIL_H

#include "pointmatcher/PointMatcher.h"
#include "definitions.h"

class Util
{
public:
    static void appendDescriptorRGBA(PM::DataPoints &dataPoints, float red,
                                     float green, float blue, float alpha);
};

#endif
