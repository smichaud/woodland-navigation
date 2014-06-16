#include "voxelgridpointcloud.h"

#include <math.h>

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
        Vector3 voxelSize) : voxelSize(voxelSize) {
    try{
        getGridMinCorner(voxelSize, dataPoints.features);
        getGridMaxCorner(voxelSize, dataPoints.features);

        Vector3 nbVoxel = (maxCorner.array() - minCorner.array())/
                voxelSize.array() + Vector3::Ones().array();

        voxels.resize(nbVoxel[xIndice]);
        for (int i = 0; i < nbVoxel[xIndice]; ++i) {
            voxels[i].resize(nbVoxel[yIndice]);

            for (int j = 0; j < nbVoxel[yIndice]; ++j)
                voxels[i][j].resize(nbVoxel[zIndice]);
        }

        setVoxelsCorners(voxelSize, nbVoxel);
        buildVoxelGridStructure(voxelSize);
    } catch(exception& e){
        cerr << "An error occured : " << e.what() << endl;
    }
}

Vector3 VoxelGridPointCloud::getPointsBoundingBoxMin(
        const PM::Matrix &features) {
    Vector3 pointsBoundingBoxMin;

    pointsBoundingBoxMin(xIndice) = features.row(xIndice).minCoeff();
    pointsBoundingBoxMin(yIndice) = features.row(yIndice).minCoeff();
    pointsBoundingBoxMin(zIndice) = features.row(zIndice).minCoeff();

    return pointsBoundingBoxMin;
}

Vector3 VoxelGridPointCloud::getPointsBoundingBoxMax(
        const PM::Matrix &features) {
    Vector3 pointsBoundingBoxMax;

    pointsBoundingBoxMax(xIndice) = features.row(xIndice).maxCoeff();
    pointsBoundingBoxMax(yIndice) = features.row(yIndice).maxCoeff();
    pointsBoundingBoxMax(zIndice) = features.row(zIndice).maxCoeff();

    return pointsBoundingBoxMax;
}

void VoxelGridPointCloud::getGridMinCorner(const Vector3 &voxelSize,
                                           const PM::Matrix& features) {
    Vector3 pointsMin = this->getPointsBoundingBoxMin(features);

    minCorner[xIndice] = floor(pointsMin[xIndice]/voxelSize[xIndice])
            *voxelSize[xIndice];
    minCorner[yIndice] = floor(pointsMin[yIndice]/voxelSize[yIndice])
            *voxelSize[yIndice];
    minCorner[zIndice] = floor(pointsMin[zIndice]/voxelSize[zIndice])
            *voxelSize[zIndice];
}

void VoxelGridPointCloud::getGridMaxCorner(const Vector3 &voxelSize,
                                           const PM::Matrix& features) {
    Vector3 pointsMax = this->getPointsBoundingBoxMax(features);

    maxCorner[xIndice] = floor(pointsMax[xIndice]/voxelSize[xIndice])
            *voxelSize[xIndice];
    maxCorner[yIndice] = floor(pointsMax[yIndice]/voxelSize[yIndice])
            *voxelSize[yIndice];
    maxCorner[zIndice] = floor(pointsMax[zIndice]/voxelSize[zIndice])
            *voxelSize[zIndice];
}

void VoxelGridPointCloud::setVoxelsCorners(Vector3 voxelSize,
                                           Vector3 nbVoxel) {
    for (int i = 0; i < nbVoxel[xIndice] ; ++i) {
        for (int j = 0; j <  nbVoxel[yIndice] ; ++j) {
            for (int k = 0; k < nbVoxel[zIndice] ; ++k) {
                Voxel &voxel = voxels[i][j][k];
                Vector3 indices;
                indices << i,j,k;
                voxel.setLowerCorner(minCorner.array()+(indices.array()
                                                        *voxelSize.array()));
            }
        }
    }
}

void VoxelGridPointCloud::buildVoxelGridStructure(Vector3 voxelSize)
{
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

PM::DataPoints VoxelGridPointCloud::getSinglePointPerVoxel() {
    int nbPoints = voxels.size()*(voxels[0]).size()*
            (voxels[0][0]).size();

    PM::DataPoints outputDataPoints;
    PM::Matrix xMatrix(1,nbPoints);
    PM::Matrix yMatrix(1,nbPoints);
    PM::Matrix zMatrix(1,nbPoints);
    PM::Matrix padMatrix(1,nbPoints);

    //    PM::Matrix colors(4,nbPoints);

    //    outputDataPoints.addFeature("x", xMatrix);
    //    outputDataPoints.addFeature("y", yMatrix);
    //    outputDataPoints.addFeature("z", zMatrix);
    //    outputDataPoints.addFeature("pad", padMatrix);

    //    outputDataPoints.addDescriptor("color", colors);

    //    int nbMaxPoints = 7528;

    //    int column = 0;
    //    for (int i = 0; i < voxels.size() ; ++i) {
    //        for (int j = 0; j <  (voxels[0]).size() ; ++j) {
    //            for (int k = 0; k < (voxels[0][0]).size() ; ++k) {
    //                float x = minCornerX + voxelSize*i + voxelSize/2;
    //                float y = minCornerY + voxelSize*j + voxelSize/2;
    //                float z = minCornerZ + voxelSize*k + voxelSize/2;
    //                float pad = 1;

    //                int nbPoints = (voxelGrid[i][j][k]).getNbPoints();
    //                float red = 0;
    //                float green = 0;
    //                float blue = 0;
    //                float alpha = 1;

    //                if (nbPoints == 0) {
    //                    alpha = 0;
    //                } else if (nbPoints < 10) {
    //                    alpha = 0.25;
    //                } else if (nbPoints < 25) {
    //                    alpha = 0.5;
    //                } else if (nbPoints < 75) {
    //                    alpha = 0.75;
    //                } else {
    //                    alpha = 1.0;
    //                }

    //                outputDataPoints.features.col(column)
    //                        << x,y,z,pad;

    //                outputDataPoints.getDescriptorViewByName("color").col(column)
    //                        << red,green,blue,alpha;

    //                ++column;
    //            }
    //        }
    //    }


    return outputDataPoints;
}
