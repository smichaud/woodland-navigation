#ifndef VOXEL_H
#define VOXEL_H

#include "pointmatcher/PointMatcher.h"

typedef PointMatcher<float> PM;

class Voxel
{
private:
    float lowerCornerX;
    float lowerCornerY;
    float lowerCornerZ;
    int nbPoints;

public:
    Voxel(float lowerCornerX=0,
          float lowerCornerY=0,
          float lowerCornerZ=0,
          int nbPoints=0);

    float getLowerCornerX() const;
    void setLowerCornerX(float value);
    float getLowerCornerY() const;
    void setLowerCornerY(float value);
    float getLowerCornerZ() const;
    void setLowerCornerZ(float value);
    int getNbPoints() const;
    void setNbPoints(int value);

    void incrementNbPoints();
};

#endif
