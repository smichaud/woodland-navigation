#include "voxel.h"

Voxel::Voxel(float cornerX, float cornerY, float cornerZ, int nbPoints) {
    this->lowerCorner << cornerX, cornerY, cornerZ;
    this->nbPoints = nbPoints;
}


void Voxel::incrementNbPoints()
{
    nbPoints++;
}
