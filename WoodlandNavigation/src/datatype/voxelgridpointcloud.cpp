#include "voxelgridpointcloud.h"

#include <math.h>
#include "mathutil.h"

using namespace std;

const used_type VoxelGridPointCloud::minPadding = static_cast<used_type>(0.001);

VoxelGridPointCloud::VoxelGridPointCloud() :
    completeDataPoints(), pointCloudBoundingBoxMin(Vector3::Zero()),
    pointCloudBoundingBoxMax(Vector3::Zero()), voxelSize(Vector3::Zero()),
    nbOfVoxels(Vector3uli::Zero()), minVoxelLowerCorner(Vector3::Zero()),
    maxVoxelLowerCorner(Vector3::Zero()), voxels() {
}

VoxelGridPointCloud::VoxelGridPointCloud(
        const PM::DataPoints &dataPoints,used_type voxelSizeX,
        used_type voxelSizeY, used_type voxelSizeZ) :
    completeDataPoints(dataPoints) {
    voxelSize << voxelSizeX, voxelSizeY, voxelSizeZ;
    buildVoxelGridPointCloud();
}

VoxelGridPointCloud::VoxelGridPointCloud(
        const PM::DataPoints &dataPoints, const Vector3 &voxelSize) :
    completeDataPoints(dataPoints), voxelSize(voxelSize){
    buildVoxelGridPointCloud();
}

void VoxelGridPointCloud::buildVoxelGridPointCloud() {
    try{
        this->computePointCloudBoundingBox();
        this->verifyVoxelSize();
        this->computeNbOfVoxels();
        this->computeGridMinMaxCorners();

        this->initVoxels();

        this->buildVoxels();
    } catch(exception& e){
        cerr << "Unable to build the VoxelGridPointCloud : "
             << e.what() << endl;
    }
}

void VoxelGridPointCloud::computePointCloudBoundingBox() {
    PM::Matrix &features = this->completeDataPoints.features;
    this->pointCloudBoundingBoxMin =
            features.rowwise().minCoeff().head(3);
    this->pointCloudBoundingBoxMax =
            features.rowwise().maxCoeff().head(3);
}

void VoxelGridPointCloud::verifyVoxelSize() {
    Vector3 defaultSize = pointCloudBoundingBoxMax - pointCloudBoundingBoxMin
            + Vector3::Ones()*minPadding;
    if(voxelSize[xIndice] <= 0) {
        voxelSize[xIndice] = defaultSize[xIndice];
    }
    if(voxelSize[yIndice] <= 0) {
        voxelSize[yIndice] = defaultSize[yIndice];
    }
    if(voxelSize[zIndice] <= 0) {
        voxelSize[zIndice] = defaultSize[zIndice];
    }
}

void VoxelGridPointCloud::computeNbOfVoxels() {
    Vector3 nbVoxelsDecimal =
            (this->pointCloudBoundingBoxMax - this->pointCloudBoundingBoxMin).
            cwiseQuotient(voxelSize);

    Vector3 nbVoxels = Vector3::Ones()
            + MathUtil::floorVector(nbVoxelsDecimal);

    this->nbOfVoxels = MathUtil::convertToIndice(nbVoxels);
}

void VoxelGridPointCloud::computeGridMinMaxCorners() {
    Vector3 nbVoxelsDecimal =
            (this->pointCloudBoundingBoxMax - this->pointCloudBoundingBoxMin).
            cwiseQuotient(voxelSize);
    Vector3 nbVoxels = this->nbOfVoxels.cast<used_type>();
    Vector3 padding = ((nbVoxels - nbVoxelsDecimal)/2).
            cwiseProduct(voxelSize);

    this->minVoxelLowerCorner = this->pointCloudBoundingBoxMin - padding;
    this->maxVoxelLowerCorner = this->pointCloudBoundingBoxMin
            + (nbVoxels - Vector3::Ones()).cwiseProduct(voxelSize);
}

void VoxelGridPointCloud::initVoxels() {
    this->voxels.resize(this->nbOfVoxels[xIndice]);
    for (unsigned long int i = 0; i < this->nbOfVoxels[xIndice]; ++i) {
        this->voxels[i].resize(this->nbOfVoxels[yIndice]);

        for (unsigned long int j = 0; j < this->nbOfVoxels[yIndice]; ++j)
            this->voxels[i][j].resize(this->nbOfVoxels[zIndice]);
    }
}

void VoxelGridPointCloud::buildVoxels() {
    unsigned long int nbOfPoints = completeDataPoints.features.cols();
    for(unsigned long int i = 0 ; i < nbOfPoints; ++i){
        Vector3 point = completeDataPoints.features.col(i).head(3);
        Vector3uli indice = this->getVoxelIndice(point);

        Voxel &voxel =
                this->voxels[indice[xIndice]][indice[yIndice]][indice[zIndice]];
        voxel.addPointIndices(i);
    }
}

Vector3 VoxelGridPointCloud::getVoxelSize() const{
    return this->voxelSize;
}

Vector3uli VoxelGridPointCloud::getNbOfVoxels() const {
    return this->nbOfVoxels;
}

Vector3uli VoxelGridPointCloud::getVoxelIndice(Vector3 pointPosition) {
    Vector3 gridIndice = MathUtil::floorVector(
                (pointPosition-minVoxelLowerCorner).cwiseQuotient(voxelSize));

    return MathUtil::convertToIndice(gridIndice);
}
