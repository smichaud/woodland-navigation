#include "voxelcloud.h"

#include <math.h>

using namespace std;

const int VoxelCloud::xIndice = 0;
const int VoxelCloud::yIndice = 1;
const int VoxelCloud::zIndice = 2;


VoxelCloud::VoxelCloud(PM::DataPoints &dataPoints, float voxelSize) {
    this->voxelSize = voxelSize;

    try{
        PM::Matrix pointCloud = dataPoints.features;

        findPointCloudBoundingBox(pointCloud);

        minCornerX = floor(minPointX/voxelSize);
        maxCornerX = floor(maxPointX/voxelSize);
        int nbVoxelX = maxCornerX - minCornerX + 1;
        
        minCornerY = floor(minPointY/voxelSize);
        maxCornerY = floor(maxPointY/voxelSize);
        int nbVoxelY = maxCornerY - minCornerY + 1;
        
        minCornerZ = floor(minPointZ/voxelSize);
        maxCornerZ = floor(maxPointZ/voxelSize);
        int nbVoxelZ = maxCornerZ - minCornerZ + 1;

        voxelGrid.resize(nbVoxelX);
        for (int i = 0; i < nbVoxelX; ++i) {
          voxelGrid[i].resize(nbVoxelY);

          for (int j = 0; j < nbVoxelY; ++j)
            voxelGrid[i][j].resize(nbVoxelZ);
        }

        //Set the corner for each voxel
        for (int i = 0; i < nbVoxelX ; ++i) {
            for (int j = 0; j <  nbVoxelY ; ++j) {
                for (int k = 0; k < nbVoxelZ ; ++k) {
                    Voxel voxel = voxelGrid[i][j][k];
                    voxel.setLowerCornerX(minCornerX+i*voxelSize);
                    voxel.setLowerCornerY(minCornerY+j*voxelSize);
                    voxel.setLowerCornerZ(minCornerZ+k*voxelSize);
                }
            }
        }

        // Set nbPoints for each voxel
        int nbMaxPoints = 0;
        for(int i = 0 ; i < pointCloud.cols(); ++i){
            float x = pointCloud(xIndice, i);
            float y = pointCloud(yIndice, i);
            float z = pointCloud(zIndice, i);

            int xVoxelIndice = floor((x-minCornerX)/voxelSize);
            int yVoxelIndice = floor((y-minCornerY)/voxelSize);
            int zVoxelIndice = floor((z-minCornerZ)/voxelSize);

            Voxel *voxel =
                    &(voxelGrid[xVoxelIndice][yVoxelIndice][zVoxelIndice]);
            voxel->incrementNbPoints();

            if(voxel->getNbPoints() > nbMaxPoints) {
                nbMaxPoints = voxel->getNbPoints();
            }
        }
        cout << "The maximum number of points in a voxel is : " << nbMaxPoints
                << endl;
    }
    catch(exception& e){
        cerr << "An error occured : " << e.what() << endl;
    }
}

void VoxelCloud::findPointCloudBoundingBox(PM::Matrix pointCloud)
{
    minPointX = pointCloud.row(xIndice).minCoeff();
    maxPointX = pointCloud.row(xIndice).maxCoeff();

    minPointY = pointCloud.row(yIndice).minCoeff();
    maxPointY = pointCloud.row(yIndice).maxCoeff();

    minPointZ = pointCloud.row(zIndice).minCoeff();
    maxPointZ = pointCloud.row(zIndice).maxCoeff();
}

PM::DataPoints VoxelCloud::getSinglePointPerVoxel() {
    int nbPoints = voxelGrid.size()*(voxelGrid[0]).size()*
            (voxelGrid[0][0]).size();

    PM::DataPoints outputDataPoints;
    PM::Matrix xMatrix(1,nbPoints);
    PM::Matrix yMatrix(1,nbPoints);
    PM::Matrix zMatrix(1,nbPoints);
    PM::Matrix padMatrix(1,nbPoints);

    PM::Matrix colors(4,nbPoints);

    outputDataPoints.addFeature("x", xMatrix);
    outputDataPoints.addFeature("y", yMatrix);
    outputDataPoints.addFeature("z", zMatrix);
    outputDataPoints.addFeature("pad", padMatrix);

    outputDataPoints.addDescriptor("color", colors);

    int nbMaxPoints = 7528;

    int column = 0;
    for (int i = 0; i < voxelGrid.size() ; ++i) {
        for (int j = 0; j <  (voxelGrid[0]).size() ; ++j) {
            for (int k = 0; k < (voxelGrid[0][0]).size() ; ++k) {
                float x = minCornerX + voxelSize*i + voxelSize/2;
                float y = minCornerY + voxelSize*j + voxelSize/2;
                float z = minCornerZ + voxelSize*k + voxelSize/2;
                float pad = 1;

                int nbPoints = (voxelGrid[i][j][k]).getNbPoints();
                float red = 0;
                float green = 0;
                float blue = 0;
                float alpha = 1;

                if (nbPoints == 0) {
                    alpha = 0;
                } else if (nbPoints < 10) {
                    alpha = 0.25;
                } else if (nbPoints < 25) {
                    alpha = 0.5;
                } else if (nbPoints < 75) {
                    alpha = 0.75;
                } else {
                    alpha = 1.0;
                }

                outputDataPoints.features.col(column)
                        << x,y,z,pad;

                outputDataPoints.getDescriptorViewByName("color").col(column)
                        << red,green,blue,alpha;

                ++column;
            }
        }
    }


    return outputDataPoints;
}


