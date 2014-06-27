#include "voxel.h"

Voxel::Voxel() {
}

uli Voxel::getNbPoints() const {
    return this->pointIndexes.size();
}

void Voxel::addPointIndex(unsigned long index) {
    this->pointIndexes.push_back(index);
}
