#ifndef VOXEL_H
#define VOXEL_H

#include "pointmatcher/PointMatcher.h"
#include "definitions.h"

class Voxel
{
public:
    std::vector<uli> pointIndexes;

public:
    Voxel();

    uli getNbPoints() const;
    void addPointIndex(uli index);
};

#endif
