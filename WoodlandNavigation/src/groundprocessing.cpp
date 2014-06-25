#include "groundprocessing.h"

#include "mathutil.h"

void GroundProcessing::addGroundDescriptor(VoxelGridPointCloud &scene) {
    VectorX descriptorDefaultValue;
    descriptorDefaultValue.setZero(1);
    scene.addDescriptor("ground", descriptorDefaultValue);

    PM::DataPoints &dataPoints = scene.getCompleteDataPoints();
    Vector3uli nbVoxels = scene.getNbVoxels();

    std::vector<uli> groundIndices;
    for(uli i = 0; i <  nbVoxels[xIndex]; ++i) {
        for(uli j = 0; j < nbVoxels[yIndex]; ++j) {
            Voxel currentVoxel = scene.getVoxel(i,j,0);
            uli nbPointsOfVoxel = currentVoxel.getNbPoints();

            std::vector<indexAndValue> vect;
            for(uli v = 0; v < nbPointsOfVoxel ;  ++v) {
                uli pointIndex = currentVoxel.pointIndexes[v];
                used_type zValue = dataPoints.features(zIndex, pointIndex);
                vect.push_back(indexAndValue(pointIndex,zValue));
            }
            try {
                uli pi = MathUtil::getRoundedQuantileRelatedIndex(vect,0.05f);
                used_type zValue = dataPoints.features(zIndex, pi);
                if(zValue <= 0){
                    dataPoints.descriptors.
                            row(dataPoints.
                                getDescriptorStartingRow("ground"))(pi) = 1.0f;
                }

            } catch (...) {
                // The cell was probably empty
            }
        }
    }
}
