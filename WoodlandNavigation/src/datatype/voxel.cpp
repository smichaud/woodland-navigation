#include "voxel.h"

Voxel::Voxel(Vector3 lowerCorner): lowerCorner(lowerCorner) {
}

Vector3 Voxel::getLowerCorner() const {
    return lowerCorner;
}

void Voxel::setLowerCorner(const Vector3 &value) {
    lowerCorner = value;
}

void Voxel::addPointIndices(unsigned long indice) {
    this->pointIndices.push_back(indice);
}
