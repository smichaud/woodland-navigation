#include "voxelgridpointcloud.h"

#include <math.h>
#include "mathutil.h"

using namespace std;


VoxelGridPointCloud::VoxelGridPointCloud() :
    completeDataPoints(), pointCloudBoundingBoxMin(Vector3::Zero()),
    pointCloudBoundingBoxMax(Vector3::Zero()), voxelSize(Vector3::Zero()),
    nbOfVoxels(Vector3uli::Zero()), minVoxelLowerCorner(Vector3::Zero()),
    maxVoxelLowerCorner(Vector3::Zero()), voxels() {
}

VoxelGridPointCloud::VoxelGridPointCloud(
        const PM::DataPoints &dataPoints,
        used_type voxelSizeX, used_type voxelSizeY, used_type voxelSizeZ) {
    voxelSize << voxelSizeX, voxelSizeY, voxelSizeZ;
    buildVoxelGridPointCloud(voxelSize);
}

VoxelGridPointCloud::VoxelGridPointCloud(const PM::DataPoints &dataPoints,
                                         const Vector3 &voxelSize) :
    completeDataPoints(dataPoints){
    buildVoxelGridPointCloud(voxelSize);
}

void VoxelGridPointCloud::buildVoxelGridPointCloud(const Vector3 &voxelSize) {
    try{
        this->computePointCloudBoundingBox(this->completeDataPoints.features);
        this->computeNbOfVoxels();
        this->computeGridMinMaxCorners();

        this->voxels.resize(this->nbOfVoxels[xIndice]);
        for (unsigned long int i = 0; i < this->nbOfVoxels[xIndice]; ++i) {
            this->voxels[i].resize(this->nbOfVoxels[yIndice]);

            for (unsigned long int j = 0; j < this->nbOfVoxels[yIndice]; ++j)
                this->voxels[i][j].resize(this->nbOfVoxels[zIndice]);
        }

        //        this->buildVoxelGridStructure(dataPoints, nbVoxel);
    } catch(exception& e){
        cerr << "unable to build the VoxelGridPointCloud : "
             << e.what() << endl;
    }
}

void VoxelGridPointCloud::computePointCloudBoundingBox(
        const PM::Matrix &features) {
    this->pointCloudBoundingBoxMin =
            features.rowwise().minCoeff().block(0,0,3,1);
    this->pointCloudBoundingBoxMax =
            features.rowwise().maxCoeff().block(0,0,3,1);
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
    //    Vector3 nbVoxels = this->nbOfVoxels.cast<used_type>();
    //    Vector3 padding = ((nbVoxels - nbVoxelsDecimal)/2).
    //            cwiseProduct(voxelSize);

    //    this->minVoxelLowerCorner = this->pointCloudBoundingBoxMin - padding;
    //    this->maxVoxelLowerCorner = this->pointCloudBoundingBoxMin
    //            + (nbVoxels - Vector3::Ones()).cwiseProduct(voxelSize);

    //    cout << "========================================" << endl;
    //    cout << "Min: " << endl << this->pointCloudBoundingBoxMin << endl;
    //    cout << "Max: " << endl << this->pointCloudBoundingBoxMax << endl;
    //    cout << "NbOfVoxels: " << endl << this->nbOfVoxels << endl;
    //    cout << "Padding: " << endl << padding << endl;
    //    cout << "minCorner: " << endl << this->minVoxelLowerCorner << endl;
    //    cout << "maxCorner: " << endl << this->maxVoxelLowerCorner << endl;
    //    cout << "========================================" << endl;
}

Vector3 VoxelGridPointCloud::getVoxelSize() const{
    return this->voxelSize;
}

Vector3uli VoxelGridPointCloud::getNbOfVoxels() const {
    return this->nbOfVoxels;
}

void VoxelGridPointCloud::buildVoxelGridStructure(
        const PM::DataPoints &dataPoints, const Vector3 &nbVoxel) {
    //    std::vector<std::vector<std::vector<std::vector<int> > > >
    //            voxelPointIndices;

    //    voxelPoints.resize(nbVoxel[xIndice]);
    //    for (int i = 0; i < nbVoxel[xIndice]; ++i) {
    //        voxels[i].resize(nbVoxel[yIndice]);

    //        for (int j = 0; j < nbVoxel[yIndice]; ++j)
    //            voxels[i][j].resize(nbVoxel[zIndice]);
    //    }

    //    //You dont want to resize the datapoint all the time
    //    //Keep only indices, then create the newcloud
    //    completeDataPoints;

    //    int nbMaxPoints = 0;
    //    for(int i = 0 ; i < pointCloud.cols(); ++i){
    //        float x = pointCloud(xIndice, i);
    //        float y = pointCloud(yIndice, i);
    //        float z = pointCloud(zIndice, i);

    //        int xVoxelIndice = floor((x-minCornerX)/voxelSize);
    //        int yVoxelIndice = floor((y-minCornerY)/voxelSize);
    //        int zVoxelIndice = floor((z-minCornerZ)/voxelSize);

    //        Voxel *voxel =
    //                &(voxels[xVoxelIndice][yVoxelIndice][zVoxelIndice]);
    //        voxel->incrementNbPoints();

    //        if(voxel->getNbPoints() > nbMaxPoints) {
    //            nbMaxPoints = voxel->getNbPoints();
    //        }
    //    }
    //    cout << "The maximum number of points in a voxel is : " << nbMaxPoints
    //         << endl;
}

Vector3 VoxelGridPointCloud::getVoxelIndice(Vector3 pointsPosition) {
    //    Vector3i gridIndice;
    //    gridIndice[xIndice] = floor((pointsPosition[xIndice]-minCorner[xIndice])/
    //                                voxelSize);
    //    gridIndice[yIndice] = floor((pointsPosition[yIndice]-minCorner[yIndice])/
    //                                voxelSize);
    //    gridIndice[zIndice] = floor((pointsPosition[zIndice]-minCorner[zIndice])/
    //                                voxelSize);
    return Vector3::Zero();
}
