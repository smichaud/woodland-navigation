#include "voxelgridpointcloud.h"

#include <math.h>
#include "mathutil.h"

using namespace std;

const used_type VoxelGridPointCloud::minPadding = static_cast<used_type>(0.001);

VoxelGridPointCloud::VoxelGridPointCloud() :
    pointCloud(), pointCloudBoundingBoxMin(Vector3::Zero()),
    pointCloudBoundingBoxMax(Vector3::Zero()), voxelSize(Vector3::Zero()),
    nbOfVoxels(Vector3uli::Zero()), minVoxelLowerCorner(Vector3::Zero()),
    maxVoxelLowerCorner(Vector3::Zero()), voxels() {
}

VoxelGridPointCloud::VoxelGridPointCloud(PointCloud &pointCloud, used_type voxelSizeX,
        used_type voxelSizeY, used_type voxelSizeZ) :
    pointCloud(pointCloud) {
    voxelSize << voxelSizeX, voxelSizeY, voxelSizeZ;
    buildVoxelGridPointCloud();
}

VoxelGridPointCloud::VoxelGridPointCloud(PointCloud &pointCloud, const Vector3 &voxelSize) :
    pointCloud(pointCloud), voxelSize(voxelSize){
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
    PM::Matrix &features = this->pointCloud.getFeaturesRef();
    this->pointCloudBoundingBoxMin =
            features.rowwise().minCoeff().head(3);
    this->pointCloudBoundingBoxMax =
            features.rowwise().maxCoeff().head(3);
}

void VoxelGridPointCloud::verifyVoxelSize() {
    Vector3 defaultSize = pointCloudBoundingBoxMax - pointCloudBoundingBoxMin
            + Vector3::Ones()*minPadding;
    if(voxelSize[xIndex] <= 0) {
        voxelSize[xIndex] = defaultSize[xIndex];
    }
    if(voxelSize[yIndex] <= 0) {
        voxelSize[yIndex] = defaultSize[yIndex];
    }
    if(voxelSize[zIndex] <= 0) {
        voxelSize[zIndex] = defaultSize[zIndex];
    }
}

void VoxelGridPointCloud::computeNbOfVoxels() {
    Vector3 nbVoxelsDecimal =
            (this->pointCloudBoundingBoxMax - this->pointCloudBoundingBoxMin).
            cwiseQuotient(voxelSize);

    Vector3 nbVoxels = Vector3::Ones()
            + MathUtil::floorVector(nbVoxelsDecimal);

    this->nbOfVoxels = MathUtil::convertToIndex(nbVoxels);
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
    this->voxels.resize(this->nbOfVoxels[xIndex]);
    for (uli i = 0; i < this->nbOfVoxels[xIndex]; ++i) {
        this->voxels[i].resize(this->nbOfVoxels[yIndex]);

        for (uli j = 0; j < this->nbOfVoxels[yIndex]; ++j)
            this->voxels[i][j].resize(this->nbOfVoxels[zIndex]);
    }
}

void VoxelGridPointCloud::buildVoxels() {
    uli nbOfPoints = pointCloud.getNbPoints();
    for(uli i = 0 ; i < nbOfPoints; ++i){
        Vector3 point = pointCloud.getPoint(i);
        Vector3uli index = this->getVoxelIndex(point);

        Voxel &voxel =
                this->voxels[index[xIndex]][index[yIndex]][index[zIndex]];
        voxel.addPointIndex(i);
    }
}

void VoxelGridPointCloud::addDescriptor(const std::string name,
                                        VectorX descriptorDefaultValue) {
    this->pointCloud.addDescriptorInitToVector(name,
                                                       descriptorDefaultValue);
}

PointCloud &VoxelGridPointCloud::getPointCloudRef() {
    PointCloud & ref = this->pointCloud;
    return ref;
}

Vector3 VoxelGridPointCloud::getVoxelSize() const {
    return this->voxelSize;
}

Vector3uli VoxelGridPointCloud::getNbVoxels() const {
    return this->nbOfVoxels;
}

Voxel VoxelGridPointCloud::getVoxel(uli x, uli y, uli z) {
    if(x >= this->nbOfVoxels[xIndex] ||
            y >= this->nbOfVoxels[yIndex] ||
            z >= this->nbOfVoxels[zIndex]) {
        throw std::invalid_argument("Voxel index out of range");
    } else {
        return this->voxels[x][y][z];
    }

}

Vector3uli VoxelGridPointCloud::getVoxelIndex(Vector3 pointPosition) {
    Vector3 gridIndex = MathUtil::floorVector(
                (pointPosition-minVoxelLowerCorner).cwiseQuotient(voxelSize));

    return MathUtil::convertToIndex(gridIndex);
}
