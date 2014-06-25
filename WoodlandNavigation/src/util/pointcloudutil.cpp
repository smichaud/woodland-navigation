#include "pointcloudutil.h"

using namespace std;

PM::DataPoints PointCloudUtil::getSinglePointPerVoxel() {
//    long int nbPoints = voxels.size()*(voxels[0]).size()*
//            (voxels[0][0]).size();

    PM::DataPoints outputDataPoints;
//    PM::Matrix xMatrix(1,nbPoints);
//    PM::Matrix yMatrix(1,nbPoints);
//    PM::Matrix zMatrix(1,nbPoints);
//    PM::Matrix padMatrix(1,nbPoints);

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
