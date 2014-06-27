#include "groundprocessing.h"

#include "mathutil.h"

void GroundProcessing::addGroundDescriptor(VoxelGridPointCloud &scene) {
    VectorX descriptorDefaultValue;
    descriptorDefaultValue.setZero(1);
    scene.addDescriptor("ground", descriptorDefaultValue);

    PointCloud &pointCloud = scene.getPointCloudRef();
    Vector3uli nbVoxels = scene.getNbVoxels();

    std::vector<uli> groundIndices;
    for(uli i = 0; i <  nbVoxels[xIndex]; ++i) {
        for(uli j = 0; j < nbVoxels[yIndex]; ++j) {
            Voxel currentVoxel = scene.getVoxel(i,j,0);
            uli nbPointsOfVoxel = currentVoxel.getNbPoints();

            std::vector<indexAndValue> vect;
            for(uli v = 0; v < nbPointsOfVoxel ;  ++v) {
                uli pointIndex = currentVoxel.pointIndexes[v];
                used_type zValue = pointCloud.getPoint(pointIndex)[zIndex];
                vect.push_back(indexAndValue(pointIndex,zValue));
            }
            try {
                uli pi = MathUtil::getRoundedQuantileRelatedIndex(vect,0.05f);
                used_type zValue = pointCloud.getPoint(pi)[zIndex];
                if(zValue <= 0){
//                    pointCloud.descriptors.
//                            row(pointCloud.
//                                getDescriptorStartingRow("ground"))(pi) = 1.0f;
                    pointCloud.setDescriptorValue(
                                "ground", pi,
                                Eigen::Matrix<used_type,1,1>::Ones());
                }

            } catch (...) {
                // The cell was probably empty
            }
        }
    }
}
