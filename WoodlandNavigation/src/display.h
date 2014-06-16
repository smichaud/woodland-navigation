#ifndef DISPLAY_H
#define DISPLAY_H

#include "pointmatcher/PointMatcher.h"
#include "definitions.h"

class Display
{
public:
    static void printAllInfo(const PM::DataPoints &dataPoints);

    static void printNumberOfPoints(const PM::DataPoints &dataPoints);

    static void printFeatures(const PM::DataPoints &dataPoints);
    static void printDescriptors(const PM::DataPoints &dataPoints);
};

#endif
