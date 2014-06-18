#include "voxelgridpointcloud.h"

#include <math.h>
#include "mathutil.h"

using namespace std;

VoxelGridPointCloud::VoxelGridPointCloud() :
    voxelSize(Vector3::Zero()),
    minCorner(Vector3::Zero()),
    maxCorner(Vector3::Zero()),
    completeDataPoints(),
    voxels() {
}

VoxelGridPointCloud::VoxelGridPointCloud(
        const PM::DataPoints &dataPoints,
        used_type voxelSizeX, used_type voxelSizeY,
        used_type voxelSizeZ) {
    Vector3 voxelSize;
    voxelSize << voxelSizeX, voxelSizeY, voxelSizeZ;

    VoxelGridPointCloud(dataPoints, voxelSize);
}

VoxelGridPointCloud::VoxelGridPointCloud(
        const PM::DataPoints &dataPoints,
        const Vector3 &voxelSize) : voxelSize(voxelSize) {
    try{
//        this->getGridMinCorner(dataPoints.features);
//        this->getGridMaxCorner(dataPoints.features);

        Vector3uli nbVoxel;
        nbVoxel = MathUtil::convertToIndice((maxCorner.array() -
                                             minCorner.array())/
                                            this->voxelSize.array() +
                                            Vector3::Ones().array());

        this->voxels.resize(nbVoxel[xIndice]);
        for (unsigned long int i = 0; i < nbVoxel[xIndice]; ++i) {
            this->voxels[i].resize(nbVoxel[yIndice]);

            for (unsigned long int j = 0; j < nbVoxel[yIndice]; ++j)
                this->voxels[i][j].resize(nbVoxel[zIndice]);
        }

        //        this->setVoxelsCorners(voxelSize, nbVoxel);
        //        this->buildVoxelGridStructure(dataPoints, nbVoxel);
    } catch(exception& e){
        cerr << "unable to build the VoxelGridPointCloud : "
             << e.what() << endl;
    }
}

Vector3 VoxelGridPointCloud::getPointsBoundingBoxMin(
        const PM::Matrix &features) {
    return features.rowwise().minCoeff().block(0,0,3,1);
}

Vector3 VoxelGridPointCloud::getPointsBoundingBoxMax(
        const PM::Matrix &features) {
    return features.rowwise().minCoeff().block(0,0,3,1);
}

Vector3 VoxelGridPointCloud::getNbVoxel() {

}

void VoxelGridPointCloud::computeGridMinMaxCorners(const PM::Matrix& features) {
    Vector3 pointsMin = getPointsBoundingBoxMin(features);
    Vector3 pointsMax = getPointsBoundingBoxMax(features);

    minCorner = MathUtil::floorVector(pointsMin.array()/voxelSize.array()).
            array() * voxelSize.array();
}

//void VoxelGridPointCloud::setVoxelsCorners(Vector3 voxelSize,
//                                           Vector3 nbVoxel) {
//    for (int i = 0; i < nbVoxel[xIndice] ; ++i) {
//        for (int j = 0; j <  nbVoxel[yIndice] ; ++j) {
//            for (int k = 0; k < nbVoxel[zIndice] ; ++k) {
//                Voxel &voxel = voxels[i][j][k];
//                Vector3 indices;
//                indices << i,j,k;
//                voxel.setLowerCorner(minCorner.array()+(indices.array()
//                                                        *voxelSize.array()));
//            }
//        }
//    }
//}

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
