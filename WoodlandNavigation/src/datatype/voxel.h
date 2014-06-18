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
    Voxel(Vector3 lowerCorner = Vector3::Zero(), int nbPoints=0);

    void incrementNbPoints();
    Vector3 getLowerCorner() const;
    void setLowerCorner(const Vector3 &value);
};

#endif
