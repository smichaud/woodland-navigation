#include "voxel.h"

Voxel::Voxel(Vector3 lowerCorner): lowerCorner(lowerCorner) {
}

Vector3 Voxel::getLowerCorner() const {
    return lowerCorner;
}

void Voxel::setLowerCorner(const Vector3 &value) {
    lowerCorner = value;
}

uli Voxel::getNbPoints() const {
    return this->pointIndexes.size();
}

void Voxel::addPointIndex(unsigned long index) {
    this->pointIndexes.push_back(index);
}
