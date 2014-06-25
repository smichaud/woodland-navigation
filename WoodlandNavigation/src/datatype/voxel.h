#ifndef VOXEL_H
#define VOXEL_H

#include "pointmatcher/PointMatcher.h"
#include "definitions.h"

class Voxel
{
public:
    Vector3 lowerCorner;
    std::vector<uli> pointIndexes;

public:
    Voxel(Vector3 lowerCorner = Vector3::Zero());

    Vector3 getLowerCorner() const;
    void setLowerCorner(const Vector3 &value);

    uli getNbPoints() const;

    void addPointIndex(uli index);
};

#endif
