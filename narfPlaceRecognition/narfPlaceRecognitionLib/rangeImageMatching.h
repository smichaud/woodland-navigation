#ifndef RANGE_IMAGE_MATCHING_H
#define RANGE_IMAGE_MATCHING_H


#pragma GCC diagnostic ignored "-Wunused-parameter"  // Do not show warnings from ROS
#include <pcl/common/poses_from_matches.h>
#undef MEASURE_FUNCTION_TIME
#pragma GCC diagnostic warning "-Wunused-parameter"

#include "scanDatabase.h"
#include "EXTERNALS/ais3dTools/basics/timeutil.h"

/**
 * \brief Class to match point clouds based on range images
 **/
class RangeImageMatching {
  public:
    //-----CLASSES/STRUCTS-----
    struct Parameters
    {
      Parameters() : maxNoOfThreadsPerScanComparison(1), maxNoOfThreadsPerDatabaseSearch(1),
                     maxDescriptorDistance(0.05), minDistanceBetweenMatches(1.0f),
                     useKDTreeForFeatureMatching(false), maxDescriptorDistanceKdtree(36*maxDescriptorDistance),
                     icpPixelSearchRadius(3), icpNumIterations(10), icpMinNumPoints(20), icpMaxNumPoints(200),
                     maxNoOfPoseEstimatesPerScanPair(2000), maxNoOfPoseEstimatesPerScanPairRotInv(4000), maxNoOfRemainingPoseEstimatesForICP(25),
                     pixelSearchRadiusForValidationPointsScore(3), maxValidationPointError(0.5f), maxValidationPointErrorBeforeIcpFactor(4.0f),
                     useComplexValidationPointScoring(true), minValidationPointScore(0.2f),
                     minNoOfTestedValidationPoints(30), maxNoOfValidationPointsBeforeICP(20),
                     useComplexValidationPointScoringBeforeICP(false), minValidationPointScoreBeforeICP(0.2),
                     weightForSeeThroughValidationPoint(25.0), weightForKnownObstacleValidationPoint(0.5),
                     weightForUnknownObstacleValidationPoint(15.0), weightForFarRangeValidationPoint(5.0),
                     doSelfSimilarityScoreAdjustment(true)
      {}
      int maxNoOfThreadsPerScanComparison;
      int maxNoOfThreadsPerDatabaseSearch;
      float maxDescriptorDistance;
      float minDistanceBetweenMatches;
      bool useKDTreeForFeatureMatching;
      float maxDescriptorDistanceKdtree;
      int icpPixelSearchRadius;
      float icpNumIterations;
      int icpMinNumPoints;
      int icpMaxNumPoints;
      int maxNoOfPoseEstimatesPerScanPair;
      int maxNoOfPoseEstimatesPerScanPairRotInv;
      int maxNoOfRemainingPoseEstimatesForICP;
      int pixelSearchRadiusForValidationPointsScore;
      float maxValidationPointError;
      float maxValidationPointErrorBeforeIcpFactor;
      bool useComplexValidationPointScoring;
      float minValidationPointScore;
      int minNoOfTestedValidationPoints;
      int maxNoOfValidationPointsBeforeICP;
      bool useComplexValidationPointScoringBeforeICP;
      float minValidationPointScoreBeforeICP;
      float weightForSeeThroughValidationPoint;
      float weightForKnownObstacleValidationPoint;
      float weightForUnknownObstacleValidationPoint;
      float weightForFarRangeValidationPoint;
      bool doSelfSimilarityScoreAdjustment;
    };
    //-----CONSTRUCTOR&DESTRUCTOR-----
    /** Constructor */
    RangeImageMatching();
    /** Destructor */
    ~RangeImageMatching();
    
    //-----METHODS-----
    static void getEstimationError(const Eigen::Isometry3f& estimatedPose, const Eigen::Isometry3f& truePose,
                                   float& errorTranslation, float& errorOrientation);
    void getBestPoseEstimatesFromSortedMatches(const ScanDatabaseElement& scan,
                                               const std::vector<pcl::PointCorrespondences6DVector>& sortedFeatureMatchesPerScan,
                                               std::vector<pcl::PosesFromMatches::PoseEstimatesVector>& poseEstimatesPerScan) const;
    void getBestPoseEstimatesFromSortedMatches(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                                               const pcl::PointCorrespondences6DVector& sortedFeatureMatches,
                                               pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates, float minScore=-1.0f) const;
    void doCompleteFiltering(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                             pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates, float minScore=-1.0f) const;
    // Get list of best pose estimates (tranformation from scan2 to scan1)
    void getBestPoseEstimates(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                              pcl::PointCorrespondences6DVector& featureMatches,
                              pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates) const;
    // Get list of best pose estimates (tranformation from scan2 to scan1)
    void getBestPoseEstimates(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                              pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates) const;
    void getBestPoseEstimates(const ScanDatabaseElement& scan,
                              std::vector<pcl::PosesFromMatches::PoseEstimatesVector>& poseEstimatesPerScan) const;
    void getBestPoseEstimates(const ScanDatabaseElement& scan, std::vector<int> scansToCompareWith,
                              std::vector<pcl::PosesFromMatches::PoseEstimatesVector>& poseEstimatesPerScan,
                              double timeLimit=-1.0f) const;
    pcl::PosesFromMatches::PoseEstimate getBestPoseEstimate(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2) const;

    float getSelfSimilarity(const ScanDatabaseElement& scan) const;

    void sortAndApplyMinScore(pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates, float minScore=-1.0f) const;
    
    void doValidationPointFiltering(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                                    pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates,
                                    float minScore=-1.0f, float maxValidationPointError=-1.0f,
                                    bool useComplexValidationPointScoring=true, int maxNoOfValidationPoints=-1, bool checkNormals=true) const;
    float getValidationPointsScore(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                                   const Eigen::Isometry3f& pose, float minScore=-1.0f, float maxValidationPointError=-1.0f,
                                   bool useComplexValidationPointScoring=true, int maxNoOfPoints=-1, bool checkNormals=true) const;
    void checkInversePoseEstimates(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                                   pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates,
                                   float minScore=-1.0f, float maxValidationPointError=-1.0f,
                                   bool useComplexValidationPointScoring=true, int maxNoOfValidationPoints=-1) const;

    void adjustUsingSelfSimilarityScore(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                                        pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates) const;

    void filterSimilarPoses(pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates) const;

    Eigen::Isometry3f doICP(const ScanDatabaseElement& scan1, const ScanDatabaseElement::VectorOfEigenVector3f& points,
                          const Eigen::Isometry3f& initialGuess, bool useKdTree=false, int numIterations=-1,
                          float maxDistanceStart=-1.0f, float maxDistanceEnd=-1.0f, int rangeImageSearchRadius=-1) const;
    void doICP(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
               pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates, bool useKdTree=false, int numIterations=-1,
               float maxDistanceStart=-1.0f, float maxDistanceEnd=-1.0f, int rangeImageSearchRadius=-1) const;
    
    ////-----PUBLIC VARIABLES-----
    //mutable double timeForLastFeatureExtraction, timeForLastFeatureMatching, timeForLastTranformationSearch, timeForLastIcpRun;
    
    ScanDatabase scanDatabase;
    Parameters parameters;
    
    ////-----PARAMS-----
    //int addScansFromIpcToDatabase;
    //int interpolateRangeImage;
    //int filterSimilarPoses;
    //int useICP;
    //int noOfIcpIterations;
    //double minIcpDist, maxIcpDist;
    //double maxHeightDiff;
    //double maxRollPitchDiffDeg;
    //double maxYawDiffDeg;
    //double depthImageAngularResolutionDeg;
    //double featureExtractionMaxRange;
    //double depthImageMinRange;
    //double depthImageMaxAngleWidthDeg;
    //double depthImageMaxAngleHeightDeg;
    //int interestPointExtractionGaussSize;
    //double interestPointExtractionThreshold;
    //double interestPointExtractionEdgeRejection;
    //double interestPointExtractionMin3dDistance;
    //int featurePixelSize;
    //double featureWorldSize;
    //double featureMaxDescrDistPerPixel;
    //int noOfValidationPoints;
    //double validationPointsMaxDistanceError;
    //double validationPointsMinScore;
    //int maxNoOfTestedTransformationsFrom3Features;
    //int maxNoOfCreatedTransformationsFrom3Features;
    //int maxNoOfTestedTransformationsFrom2Features;
    //int maxNoOfCreatedTransformationsFrom2Features;
    //int maxNoOfTransformationsFromSingleFeatures;
    //double minDistanceBetweenFeaturesForPositionEstimation;
    //double minAngleBetweenFeatureConnectionsDeg;
    //int maxNoOfPossibleMatchesPerFeature;
    //int useKDTreeForFeatureMatching;
    //int maxNodesForBBF;
    
    //-----Member-----


};

//template <typename PointCloudWithSensorPosesType>
//void RangeImageMatching::extractRangeImage(const PointCloudWithSensorPosesType& pointCloud, DepthImagef& output, bool calculateScanBorder) const {
  //Matrix3x3f virtualDepthCameraOrientation(1,0,0,0,1,0,0,0,1);
  //Vector3f sensorPos = pointCloud.getSensorPosMean();
  
  //const vector<PointWithSensorPosf>* maxRanges=NULL;
  //if (!pointCloud.maxRanges.empty()) maxRanges = &pointCloud.maxRanges;
  
  //output.createFromPointCloud(pointCloud, DEG2RAD(depthImageAngularResolutionDeg),
                              //DEG2RAD(depthImageMaxAngleWidthDeg), DEG2RAD(depthImageMaxAngleHeightDeg),
                              //virtualDepthCameraOrientation, sensorPos, calculateScanBorder, NULL, NULL, 0, depthImageMinRange,
                              //maxRanges);
  
  //if (interpolateRangeImage)  output.fillInfHoles(1);
//}

//template <typename PointCloudWithSensorPosesType>
//void RangeImageMatching::loadPointCloudFromDisk(const char* fileName, PointCloudWithSensorPosesType& pointCloud) const {
  //pointCloud.clear();
  
  //if (pointCloud.checkFileHeader(fileName)) {
    //pointCloud.readBinary(fileName);
  //}
  //else {
    ////  Check if this file is a PointCloudWithSensorPosesf-file
    //PointCloudWithSensorPosesf tmpPointCloudWithSensorPosesf;
    //ColoredPointCloudWithSensorPosesf tmpColoredPointCloudWithSensorPosesf;
    //if (tmpPointCloudWithSensorPosesf.checkFileHeader(fileName)) {
      //cout << "File is not a "<<pointCloud.getPointCloudType()<<", but a PointCloudWithSensorPosesf.\n";
      //tmpPointCloudWithSensorPosesf.readBinary(fileName);
      //pointCloud.addPointsOfPointCloudWithSensorPoses(tmpPointCloudWithSensorPosesf);
    //}
    ////  Check if this file is a ColoredPointCloudWithSensorPosesf-file
    //else if (tmpColoredPointCloudWithSensorPosesf.checkFileHeader(fileName)) {
      //cout << "File is not a "<<pointCloud.getPointCloudType()<<", but a ColoredPointCloudWithSensorPosesf.\n";
      //tmpColoredPointCloudWithSensorPosesf.readBinary(fileName);
      //pointCloud.addPointsOfPointCloudWithSensorPoses(tmpColoredPointCloudWithSensorPosesf);
    //}
    //else {
      //cout << "Unknown file type... Reading anyway...\n";
      //tmpPointCloudWithSensorPosesf.readBinary(fileName);
      //pointCloud.addPointsOfPointCloudWithSensorPoses(tmpPointCloudWithSensorPosesf);
    //}
  //}
//}

#endif
