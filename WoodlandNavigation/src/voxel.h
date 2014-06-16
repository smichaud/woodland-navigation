#ifndef VOXEL_H
#define VOXEL_H

#include "pointmatcher/PointMatcher.h"
#include "definitions.h"

class Voxel
{
private:
    Vector3 lowerCorner;
    int nbPoints;

public:
    Voxel(float lowerCornerX=0,
          float lowerCornerY=0,
          float lowerCornerZ=0,
          int nbPoints=0);

    void incrementNbPoints();
};

#endif
