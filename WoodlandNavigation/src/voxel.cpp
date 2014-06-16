#include "voxel.h"

Voxel::Voxel(Vector3 lowerCorner, int nbPoints) : nbPoints(nbPoints) {
}

Vector3 Voxel::getLowerCorner() const
{
    return lowerCorner;
}

void Voxel::setLowerCorner(const Vector3 &value)
{
    lowerCorner = value;
}

void Voxel::incrementNbPoints()
{
    nbPoints++;
}
