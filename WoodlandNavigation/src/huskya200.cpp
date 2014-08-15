#include "huskya200.h"

#include "mathutil.h"
#include <math.h>
#include "pointcloudfilter.h"
#include "pointcloudprocessing.h"
#include "display.h"

HuskyA200::HuskyA200() :
    groundMaxAngleInRad(0.523598776f),
    minEffectiveSensorRange(1.0f),
    maxEffectiveSensorRange(6.0f),
    knnUsedForPointCloud(15),
    maxPointCloudDensity(750),
    navigationLabel("navigable"),
    label_unknown(0.5f),
    label_ground(1.0f),
    label_tree(0.0f){
}

void HuskyA200::gotPointCloud(PointCloud pointCloud) {
    this->currentPointCloud = pointCloud;
    this->currentPointCloud.addDescriptorInitToVector(
                this->navigationLabel, Scalar::Ones()*this->label_unknown);

    this->preprocessPointCloud();
    this->segmentGroud();
    this->segmentTrees();
}

void HuskyA200::preprocessPointCloud() {
    PointCloudFilter::minRadius(this->currentPointCloud,
                                this->minEffectiveSensorRange);
    PointCloudFilter::maxRadius(this->currentPointCloud,
                                this->maxEffectiveSensorRange);

    this->currentPointCloud.addDensityDescriptors(
                this->knnUsedForPointCloud);
    PointCloudFilter::maxDensity(this->currentPointCloud,
                                 this->maxPointCloudDensity);

    this->currentPointCloud.addObservationDirectionDescriptors();
    this->currentPointCloud.addKnnEigenRelatedDescriptors(
                this->knnUsedForPointCloud,
                true, true, true, false);

    this->currentPointCloud.orientSurfaceNormalsRelativeToCenter();
}

void HuskyA200::segmentGroud() {
    float colVoxelSide = 1.0f;
    VoxelGridPointCloud voxelGridPointCloud =
            this->createColumnVoxels(colVoxelSide);

    Vector3uli nbVoxels = voxelGridPointCloud.getNbVoxels();
    for(auto i = 0u; i <  nbVoxels[xIndex]; ++i) {
        for(auto j = 0u; j < nbVoxels[yIndex]; ++j) {
            Voxel currentVoxel = voxelGridPointCloud.getVoxel(i,j,0);
            segmentVoxelGround(currentVoxel);
        }
    }
}

VoxelGridPointCloud HuskyA200::createColumnVoxels(
        used_type colVoxelBaseSideSize) {
    VoxelGridPointCloud voxelGridPointCloud(
                this->currentPointCloud,
                colVoxelBaseSideSize,colVoxelBaseSideSize,0);

    return voxelGridPointCloud;
}

void HuskyA200::segmentVoxelGround(const Voxel &voxel)
{
    uli minNbPointsForProcess = 12;
    try {
        uli groundPointIndex =
                this->guessVoxelGroundHeight(voxel);
        if(voxel.getNbPoints() > minNbPointsForProcess
                && this->isPossibleGroundAngle(groundPointIndex)
                /*&& this->isPlanarRegion(groundPointIndex)*/){
            this->labelAllPointsInThresholdAsGround(voxel,
                                                    groundPointIndex,
                                                    0.1f);
        }
    } catch(mathutil::empty_container) {
    } catch(...){
        std::cerr << "A problem occured during ground segmentation"
                  << std::endl;
    }
}

uli HuskyA200::guessVoxelGroundHeight(const Voxel &voxel) const {
    float quantile = 0.05f;
    return this->getVoxelQuantileIndex(voxel, quantile);
}

uli HuskyA200::getVoxelQuantileIndex(
        const Voxel &voxel, const float quantile) const {
    uli nbPointsOfVoxel = voxel.getNbPoints();

    std::vector<indexAndValue> voxelIndexesAndZValues;
    for(auto v = 0u; v < nbPointsOfVoxel ;  ++v) {
        uli pointIndex = voxel.pointIndexes[v];
        used_type zValue = this->currentPointCloud.getPoint(pointIndex)[zIndex];
        voxelIndexesAndZValues.push_back(indexAndValue(pointIndex,zValue));
    }

    return mathutil::getRoundedQuantileRelatedIndex(voxelIndexesAndZValues,
                                                    quantile);
}

bool HuskyA200::isPossibleGroundAngle(const uli pointIndex) const {
    Vector3 zPositiveUnitVector;
    zPositiveUnitVector << 0,0,1;

    Vector3 lidarToPointUnitVector = (-1)*this->currentPointCloud.
            getDescriptorValue("observationDirections",pointIndex).normalized();

    used_type cosAngleThreshold =
            static_cast<used_type>(cos(M_PI_2 - this->groundMaxAngleInRad));

    if(lidarToPointUnitVector.dot(zPositiveUnitVector) > cosAngleThreshold) {
        return false;
    }

    return true;
}

bool HuskyA200::isPlanarRegion(const uli pointIndex) const {
    used_type minEigRatioThreshold = 3.0f;
    Vector3 eigenValues = this->currentPointCloud.
            getDescriptorValue("eigValues", pointIndex);

    std::sort(eigenValues.data(),
              eigenValues.data()+eigenValues.size());
    used_type minEigRatio = eigenValues[1]/eigenValues[0];

    if(minEigRatio < minEigRatioThreshold) {
        return false;
    }

    return true;
}

void HuskyA200::labelAllPointsInThresholdAsGround(const Voxel &voxel,
                                                  const uli groundPointIndex,
                                                  const used_type threshold) {
    used_type guessedGroundHeight =
            this->currentPointCloud.getPoint(groundPointIndex)[zIndex];
    uli nbPointsOfVoxel = voxel.getNbPoints();

    std::vector<uli> groundPointsInThreshold;
    for(auto i = 0u; i < nbPointsOfVoxel ;  ++i) {
        uli pointIndex = voxel.pointIndexes[i];
        used_type zValue =
                this->currentPointCloud.getPoint(pointIndex)[zIndex];

        if(zValue > guessedGroundHeight - threshold
                && zValue < guessedGroundHeight + threshold) {
            this->currentPointCloud.setDescriptorValue(
                        this->navigationLabel, pointIndex,
                        Scalar::Ones()*this->label_ground);
            groundPointsInThreshold.push_back(pointIndex);
        }
    }

    this->unlabelIfNotEnoughPoints(groundPointsInThreshold);
}

void HuskyA200::unlabelIfNotEnoughPoints(
        const std::vector<uli> &groundPointsInThreshold) {
    uli minNbOfPoints = 12;

    if(groundPointsInThreshold.size() < minNbOfPoints){
        for(auto index : groundPointsInThreshold) {
            this->currentPointCloud.setDescriptorValue(
                        this->navigationLabel, index,
                        Scalar::Ones()*this->label_unknown);
        }
    }
}

void HuskyA200::segmentTrees() {
    const used_type minHeight = 0.8f; //TODO this is no relative to ground yet
    const used_type maxHeight = 1.6f;
    const used_type maxDist = 0.35f;

    auto nbPoints = this->currentPointCloud.getNbPoints();
    std::vector<uli> processedPointIndexes;
    processedPointIndexes.reserve(nbPoints);
    for(auto i = 0u; i < nbPoints; ++i) {
        used_type zValue = this->currentPointCloud.getPoint(i)[zIndex];
        if(zValue > minHeight && zValue < maxHeight){
            processedPointIndexes.push_back(i);
        }
    }

    PointCloudProcessing::labelSingleLinkageClusters(this->currentPointCloud,
                                                     "cluster",
                                                     processedPointIndexes,
                                                     maxDist);
}

void HuskyA200::savePointCloudRepresentation(const std::string filename) const {
    this->currentPointCloud.save(filename);
}

