#include "voxel.h"

Voxel::Voxel(float cornerX, float cornerY, float cornerZ, int nbPoints) {
    this->lowerCornerX = cornerX;
    this->lowerCornerY = cornerY;
    this->lowerCornerZ = cornerZ;
    this->nbPoints = nbPoints;
}

float Voxel::getLowerCornerX() const{
    return lowerCornerX;
}

void Voxel::setLowerCornerX(float value){
    lowerCornerX = value;
}

float Voxel::getLowerCornerY() const{
    return lowerCornerY;
}

void Voxel::setLowerCornerY(float value){
    lowerCornerY = value;
}

float Voxel::getLowerCornerZ() const{
    return lowerCornerZ;
}

void Voxel::setLowerCornerZ(float value){
    lowerCornerZ = value;
}

int Voxel::getNbPoints() const{
    return nbPoints;
}

void Voxel::setNbPoints(int value){
    nbPoints = value;
}

void Voxel::incrementNbPoints()
{
    nbPoints++;
}
