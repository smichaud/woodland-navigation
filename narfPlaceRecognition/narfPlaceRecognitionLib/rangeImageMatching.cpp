#include "rangeImageMatching.h"

#include <iostream>
using std::cout;
#include <cmath>
#include "EXTERNALS/ais3dTools/basics/misc.h"
using Ais3dTools::deg2rad;
using Ais3dTools::get_time;
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/eigen.h>

//#include <carmen/carmen.h>
//#include "pointCloudAnalysis/positionEstimationFrom3DFeatures/positionEstimationFrom3DFeatures.h"

RangeImageMatching::RangeImageMatching() {
  //cout << (int)PossibleFeatureMatch::orderAccordingToNoOfMatches<<"\n";
  //PossibleFeatureMatch::orderAccordingToNoOfMatches = true;
  //PossibleFeatureMatch::orderAccordingToNoOfMatches = false;
}

RangeImageMatching::~RangeImageMatching() {
}

void RangeImageMatching::doValidationPointFiltering(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                                                    pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates,
                                                    float minScore, float maxValidationPointError, bool useComplexValidationPointScoring,
                                                    int maxNoOfValidationPoints, bool checkNormals) const
{
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  //MEASURE_FUNCTION_TIME;
  
  #pragma omp parallel for default(shared) num_threads(parameters.maxNoOfThreadsPerScanComparison) schedule(dynamic, 5)
  for (unsigned int poseEstimateIdx=0; poseEstimateIdx<poseEstimates.size(); ++poseEstimateIdx)
  {
    pcl::PosesFromMatches::PoseEstimate& poseEstimate = poseEstimates[poseEstimateIdx];
    Eigen::Isometry3f transformation;
    transformation.matrix() = poseEstimate.transformation.matrix();
    poseEstimate.score = getValidationPointsScore(scan1, scan2, transformation, minScore, maxValidationPointError,
                                                  useComplexValidationPointScoring, maxNoOfValidationPoints, checkNormals);
    //std::cout << PVARN(poseEstimate.score);
  }
}

float RangeImageMatching::getValidationPointsScore(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                                                   const Eigen::Isometry3f& pose, float minScore, float maxValidationPointError,
                                                   bool useComplexValidationPointScoring, int maxNoOfPoints, bool checkNormals) const
{
  if (minScore < 0.0f)
    minScore = parameters.minValidationPointScore;
  if (maxValidationPointError < 0.0f)
    maxValidationPointError = parameters.maxValidationPointError;
  if (maxNoOfPoints<0)
    maxNoOfPoints = scan2.validationPoints.size();
  else
    maxNoOfPoints = std::min(maxNoOfPoints, int(scan2.validationPoints.size()));
  
  float maxDistanceError = maxValidationPointError;
  int neighborRadius = parameters.pixelSearchRadiusForValidationPointsScore;
  float minDistanceError = 0.25f*maxDistanceError;
  float minScoreToStop = 0.75f*minScore;
  
  float score=0.0f, weightSum=0.0f;
  float maxDistanceErrorReciprocal = 1.0/maxDistanceError;
  
  Eigen::Isometry3f inversePose = pose.inverse();
  
  for (int validationPointIdx=0; validationPointIdx<maxNoOfPoints; ++validationPointIdx) {
    const Eigen::Vector3f& validationPoint = scan2.validationPoints[validationPointIdx];
    Eigen::Vector3f transformedPoint = pose*validationPoint;
    int imageX, imageY;
    float shouldBeRange;
    scan1.rangeImage.getImagePoint(transformedPoint, imageX, imageY, shouldBeRange);
    
    float pointScore = 0.0f,
          currentWeight = 1.0f;
    
    bool mightBeBlockedByKnownObstacle   = false,
         mightBeBlockedByUnknownObstacle = false,
         mightBeSeeThrough               = false,
         mightBeUnobserved               = false,
         mightBeOutOfRange               = false;
    
    int bestX, bestY;
    for (int x2=imageX-neighborRadius; x2<=imageX+neighborRadius; ++x2) {
      for (int y2=imageY-neighborRadius; y2<=imageY+neighborRadius; ++y2) {
        const pcl::PointWithRange& rangeImagePoint = scan1.rangeImage.getPoint(x2, y2);
        //std::cout << " "<<rangeImagePoint.range<<"-"<<shouldBeRange<<"("<<maxDistanceError<<"), "<<std::flush;
        if (isinf(rangeImagePoint.range)) {
          if (rangeImagePoint.range < 0)
            mightBeUnobserved = true;
          else
            mightBeOutOfRange = true;
          //std::cout << "1"<<std::flush;
          continue;
        }
        float distanceError = rangeImagePoint.range - shouldBeRange;
        
        if (fabs(distanceError) > maxDistanceError) {
          if (useComplexValidationPointScoring) {
            if (distanceError < 0.0f) {
              Eigen::Vector3f backTransformedPoint = inversePose*rangeImagePoint.getVector3fMap();
              pcl::PointWithRange originalBackTransformedPoint;
              float shouldBeRange2 = scan2.rangeImage.checkPoint(backTransformedPoint, originalBackTransformedPoint);
              if (std::isfinite(originalBackTransformedPoint.range) && fabsf(shouldBeRange2-originalBackTransformedPoint.range)<maxDistanceError)
                mightBeBlockedByKnownObstacle = true;
              else
                mightBeBlockedByUnknownObstacle = true;
            }
            else
              mightBeSeeThrough = true;
          }
          //std::cout << "2"<<std::flush;
          continue;
        }
        
        if (fabs(distanceError) < minDistanceError)
          distanceError = 0.0;
        
        float imagePosErrorFactor = 0.75f + (0.25 / (hypot(imageX-x2, imageY-y2)+1.0f));
        float currentPointQuality = (1.0 - (fabs(distanceError)*maxDistanceErrorReciprocal)) * imagePosErrorFactor;
        
        if (currentPointQuality > pointScore) {
          pointScore = currentPointQuality;
          bestX = x2;
          bestY = y2;
        }
        //std::cout << "3"<<std::flush;
      }
    }
    
    currentWeight = 1.0f;
    
    if (pointScore == 0) {
      if (useComplexValidationPointScoring) {  // Handle the different cases of wrong points
        currentWeight = INFINITY;
        if (mightBeBlockedByKnownObstacle || mightBeUnobserved)
          currentWeight = std::min(currentWeight, parameters.weightForKnownObstacleValidationPoint);
        if (mightBeBlockedByUnknownObstacle)
          currentWeight = std::min(currentWeight, parameters.weightForUnknownObstacleValidationPoint);
        if (mightBeSeeThrough)
          currentWeight = std::min(currentWeight, parameters.weightForSeeThroughValidationPoint);
        if (mightBeOutOfRange) {
          float originalRange = (scan2.rangeImage.getSensorPos()-validationPoint).norm();
          if (originalRange >= shouldBeRange)
            currentWeight = std::min(currentWeight, parameters.weightForSeeThroughValidationPoint);
          else
            currentWeight = std::min(currentWeight, parameters.weightForFarRangeValidationPoint);
        }
        //cout << PVARN(currentWeight);
      }
    }
    else {
      // Check if normal points in same direction
      if (checkNormals) {
        //std::cout << "Checking normal.\n";
        Eigen::Vector3f normal2 = pose.linear() * scan2.validationPoints.normals[validationPointIdx];
        Eigen::Vector3f normal1;
        scan1.rangeImage.getNormalForClosestNeighbors(bestX, bestY, normal1, 2);
        if (normal1.dot(normal2) < 0) {
          currentWeight = std::max(currentWeight, 1.0f);
          pointScore = 0.0f;
        }
      }
    }

    //std::cout << "Point score "<<pointScore<<", weight "<<currentWeight<<".\n";
    
    weightSum += currentWeight;
    score += currentWeight * (pointScore-score)/weightSum;
    
    if ((int)validationPointIdx>=parameters.minNoOfTestedValidationPoints && score<minScoreToStop)
      break;
  }
  return score;
}

Eigen::Isometry3f RangeImageMatching::doICP(const ScanDatabaseElement& scan1, const ScanDatabaseElement::VectorOfEigenVector3f& points,
                                            const Eigen::Isometry3f& initialGuess, bool useKdTree, int numIterations, float maxDistanceStart,
                                            float maxDistanceEnd, int rangeImageSearchRadius) const
{
  //MEASURE_FUNCTION_TIME;
  
  if (maxDistanceStart < 0)
    maxDistanceStart=4.0f*parameters.maxValidationPointError;
  if (maxDistanceEnd < 0)
    maxDistanceEnd=1.0f*parameters.maxValidationPointError;
  if (numIterations < 0)
    numIterations = parameters.icpNumIterations;
  
  int maxSearchRadius = rangeImageSearchRadius;
  if (maxSearchRadius < 0)
    maxSearchRadius = parameters.icpPixelSearchRadius;
  
  Eigen::Isometry3f ret = initialGuess;
  
  float max_distance = maxDistanceStart, 
        max_distance_reduction = (maxDistanceStart-maxDistanceEnd)/float (numIterations);
  
  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;
  
  pcl::TransformationFromCorrespondences transformation_from_correspondeces;
  for (int iteration=1; iteration<=numIterations; ++iteration)
  {
    int search_radius = maxSearchRadius;
    if (iteration <= 2)
      ++search_radius;
    float max_distance_squared = max_distance*max_distance;
    transformation_from_correspondeces.reset ();
    int noOfPoints = parameters.icpMaxNumPoints;
    if (iteration<numIterations) {
      float factor = 1.0f - powf(float(iteration-1)/float(numIterations), 2);
      noOfPoints -= lrintf(factor*(parameters.icpMaxNumPoints-parameters.icpMinNumPoints));
    }
    noOfPoints = std::min(noOfPoints, int(points.size()));
    
    if (useKdTree) {
      std::cerr << "useKdTree currently not valid!\n";
      
      ////cout << PVARC(iteration)<<PVARN(noOfPoints);
      //for (int point_idx=0; point_idx < noOfPoints; ++point_idx)
      //{
        //const Eigen::Vector3f& point = points[point_idx];
        //Eigen::Vector3f transformed_point = ret * point;
        
        ////std::pair<ScanDatabaseElement::KDTreeForVector3f::const_iterator, float> kdTreeRet = 
          ////scan1.kdTreeForRangeImage.find_nearest (transformed_point, max_distance);
        ////if (kdTreeRet.first != scan1.kdTreeForRangeImage.end()) {
          //////cout << kdTreeRet.second<<", ";
          ////transformation_from_correspondeces.add (point, *kdTreeRet.first);
        ////}
        
        //pcl::PointWithRange tmpPoint;
        //tmpPoint.getVector3fMap() = transformed_point;
        //k_indices.clear();
        //k_sqr_distances.clear();
        //if (const_cast<ScanDatabaseElement*>(&scan1)->kdTreeForRangeImage.nearestKSearch(tmpPoint, 1, k_indices, k_sqr_distances) > 0) {
          //if (k_sqr_distances[0] < max_distance_squared)
            //transformation_from_correspondeces.add (point, scan1.rangeImage.points[k_indices[0]].getVector3fMap());
        //}
      //}
    }
    //else
    {
       //cout << PVARC(iteration)<<PVARN(noOfPoints);
      for (int point_idx=0; point_idx < noOfPoints; ++point_idx)
      {
        const Eigen::Vector3f& point = points[point_idx];
        Eigen::Vector3f transformed_point = ret * point;
        
        int x,y;
        //std::cout << point.x()<<" "<<point.y()<<" "<<point.z()<<" - "
                  //<< transformed_point.x()<<" "<<transformed_point.y()<<" "<<transformed_point.z()<<"\n";
        scan1.rangeImage.getImagePoint (transformed_point, x, y);
        float closest_distance = max_distance_squared;
        const pcl::PointWithRange* closest_point = NULL;
        for (int y2=y-search_radius; y2<=y+search_radius; ++y2)
        {
          for (int x2=x-search_radius; x2<=x+search_radius; ++x2)
          {
            const pcl::PointWithRange& neighbor = scan1.rangeImage.getPoint (x2, y2);
            if (!std::isfinite (neighbor.range))
              continue;
            float distance = (transformed_point-neighbor.getVector3fMap ()).squaredNorm ();
            if (distance < closest_distance)
            {
              closest_distance = distance;
              closest_point = &neighbor;
            }
          }
        }
        if (closest_point != NULL)
        {
          //cout << PVARN (closest_distance);
          transformation_from_correspondeces.add (point, closest_point->getVector3fMap());
        }
      }
    }
    //cout << PVARC(iteration)<<PVARN(transformation_from_correspondeces.getNoOfSamples ());
    if (transformation_from_correspondeces.getNoOfSamples () < 3)
      return ret;
    // TODO: check if change
    ret.matrix() = transformation_from_correspondeces.getTransformation().matrix();
    //cout << ret.matrix()<<"\n";
    
    max_distance -= max_distance_reduction;
  }
  
  //cout << PVARN (initial_guess.matrix ())<<PVARN (ret.matrix ());
  
  return ret;
}

void RangeImageMatching::doICP(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                               pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates, bool useKdTree,
                               int numIterations, float maxDistanceStart, float maxDistanceEnd, int rangeImageSearchRadius) const
                               //float minScore, float maxValidationPointError, bool useComplexValidationPointScoring, int maxNoOfValidationPoints) const
{
  //MEASURE_FUNCTION_TIME;

  if (parameters.icpNumIterations == 0)
    return;
  

  //pcl::PosesFromMatches::PoseEstimatesVector poseEstimatesAfterIcp = poseEstimates;
  //int improvedByIcpCounter = 0;
  //#pragma omp parallel for default(shared) num_threads(parameters.maxNoOfThreadsPerScanComparison) schedule(dynamic, 100)
  for (size_t poseEstimateIdx=0; poseEstimateIdx<poseEstimates.size(); ++poseEstimateIdx)
  {
    pcl::PosesFromMatches::PoseEstimate& poseEstimate = poseEstimates[poseEstimateIdx];
    pcl::PosesFromMatches::PoseEstimate poseEstimateAfterIcp = poseEstimate;
    Eigen::Isometry3f transformation;
    transformation.matrix() = poseEstimate.transformation.matrix();
    poseEstimateAfterIcp.transformation = doICP(scan1, scan2.validationPoints, transformation, useKdTree,
                                                numIterations, maxDistanceStart, maxDistanceEnd, rangeImageSearchRadius);
    //poseEstimateAfterIcp.score = getValidationPointsScore(scan1, scan2, poseEstimateAfterIcp.transformation,
                                                          //minScore, maxValidationPointError, useComplexValidationPointScoring, maxNoOfValidationPoints);
    if (poseEstimateAfterIcp.score >= poseEstimate.score) {
      //cout << "ICP improved result ("<<poseEstimate.score<<"->"<<poseEstimateAfterIcp.score<<").\n";
      poseEstimate = poseEstimateAfterIcp;
      //#pragma omp singleton
      //++improvedByIcpCounter;
    }
    else {
      //cout << "ICP did not improved result ("<<poseEstimate.score<<"->"<<poseEstimateAfterIcp.score<<").\n";
    }
  }
  //cout << improvedByIcpCounter << "/" << poseEstimates.size() << " poses were improved by ICP.\n";
}

void RangeImageMatching::filterSimilarPoses(pcl::PosesFromMatches::PoseEstimatesVector& pose_estimates) const
{
  //MEASURE_FUNCTION_TIME;
  
  float minDistance = 4.0f*parameters.maxValidationPointError,
        minDistanceSquared = minDistance*minDistance;
  float maxAngularError = deg2rad(10.0f),
        cosMaxAngularError = cosf(maxAngularError);
  bool* keep = new bool[pose_estimates.size()];
  SET_ARRAY(keep, true, pose_estimates.size());
  for (int poseIdx1=0; poseIdx1<int(pose_estimates.size()); ++poseIdx1)
  {
    if (!keep[poseIdx1])
      continue;
    const pcl::PosesFromMatches::PoseEstimate& pose_estimate1 = pose_estimates[poseIdx1];
    Eigen::Vector3f center1 = pose_estimate1.transformation * Eigen::Vector3f(0,0,0);
    for (int poseIdx2=0; poseIdx2<int(pose_estimates.size()); ++poseIdx2)
    {
      if (poseIdx1==poseIdx2 || !keep[poseIdx2])
        continue;
      const pcl::PosesFromMatches::PoseEstimate& pose_estimate2 = pose_estimates[poseIdx2];
      Eigen::Vector3f center2 = pose_estimate2.transformation * Eigen::Vector3f(0,0,0);
      float squared_distance = (center2-center1).squaredNorm();
      if (squared_distance >= minDistanceSquared)
        continue;
      float cosAngularErrorX = (pose_estimate1.transformation.rotation()*Eigen::Vector3f(1,0,0)).dot(
                                pose_estimate2.transformation.rotation()*Eigen::Vector3f(1,0,0));
      if (cosAngularErrorX <= cosMaxAngularError)
        continue;
      float cosAngularErrorY = (pose_estimate1.transformation.rotation()*Eigen::Vector3f(0,1,0)).dot(
                                pose_estimate2.transformation.rotation()*Eigen::Vector3f(0,1,0));
      if (cosAngularErrorY <= cosMaxAngularError)
        continue;
      float cosAngularErrorZ = (pose_estimate1.transformation.rotation()*Eigen::Vector3f(0,0,1)).dot(
                                pose_estimate2.transformation.rotation()*Eigen::Vector3f(0,0,1));
      if (cosAngularErrorZ <= cosMaxAngularError)
        continue;
      
      if (pose_estimate1.score > pose_estimate2.score)
        keep[poseIdx2] = false;
      else {
        keep[poseIdx1] = false;
        break;
      }
    }
  }
  
  int lastUnusedIdx = 0;
  for (int poseIdx=0; poseIdx<int(pose_estimates.size()); ++poseIdx)
    if (keep[poseIdx])
      pose_estimates[lastUnusedIdx++] = pose_estimates[poseIdx];
  delete[] keep;
  
  //cout << lastUnusedIdx<<"/"<<pose_estimates.size()<<" poses left after filtering of similar poses.\n";
  
  pose_estimates.resize(lastUnusedIdx);
}

void RangeImageMatching::sortAndApplyMinScore(pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates, float minScore) const {
  if (minScore < 0.0f)
    minScore = parameters.minValidationPointScore;
  std::sort(poseEstimates.begin(), poseEstimates.end(), pcl::PosesFromMatches::PoseEstimate::IsBetter());
  for (unsigned int poseEstimateIdx=0; poseEstimateIdx<poseEstimates.size(); ++poseEstimateIdx)
    if (poseEstimates[poseEstimateIdx].score < minScore)
      poseEstimates.resize(poseEstimateIdx);
}

void RangeImageMatching::doCompleteFiltering(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                                             pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates, float minScore) const
{
  //cout << "\n"<<__PRETTY_FUNCTION__<<": called.\n";
  //MEASURE_FUNCTION_TIME;
  
  float maxValidationPointErrorBeforeICP = parameters.maxValidationPointErrorBeforeIcpFactor * parameters.maxValidationPointError;
  
  if (minScore < 0.0f)
    minScore = parameters.minValidationPointScore;
  
  //std::cout << "Now filtering "<<poseEstimates.size()<<" pose estimates with minimum score "<<minScore<<".\n";

  //std::cout << PVARC(parameters.minValidationPointScoreBeforeICP) << PVARC(parameters.maxValidationPointErrorBeforeIcpFactor)
            //<< PVARC(parameters.maxValidationPointError) << PVARN(maxValidationPointErrorBeforeICP);
  
  double timePoseScoring = -get_time();
  doValidationPointFiltering(scan1, scan2, poseEstimates, parameters.minValidationPointScoreBeforeICP,
                             maxValidationPointErrorBeforeICP, parameters.useComplexValidationPointScoringBeforeICP,
                             parameters.maxNoOfValidationPointsBeforeICP, false);
  sortAndApplyMinScore(poseEstimates, parameters.minValidationPointScoreBeforeICP);
  //std::cout << poseEstimates.size()<<" pose estimates left after initial validation point scoring.\n";
  filterSimilarPoses(poseEstimates);
  
  //std::cout << poseEstimates.size()<<" pose estimates left after filtering similar poses.\n";
  
  poseEstimates.resize(std::min(int(poseEstimates.size()), parameters.maxNoOfRemainingPoseEstimatesForICP));
  timePoseScoring += get_time();
  
  //std::cout << poseEstimates.size()<<" pose estimates left for ICP.\n";
  
  double timeICP = -get_time();
  doICP(scan1, scan2, poseEstimates);
  timeICP += get_time();
  
  double timeRest = -get_time();
  double timePoseScoring2 = -get_time();
  doValidationPointFiltering(scan1, scan2, poseEstimates, minScore, parameters.maxValidationPointError,
                             parameters.useComplexValidationPointScoring, -1);
  timePoseScoring2 += get_time();
  
  sortAndApplyMinScore(poseEstimates, minScore);

  //std::cout << poseEstimates.size()<<" pose estimates left after second validation point scoring.\n";
  
  filterSimilarPoses(poseEstimates);
  //poseEstimates.resize(std::min(int(poseEstimates.size()), 10));

  //std::cout << poseEstimates.size()<<" pose estimates left after filtering similar poses.\n";
  
  double timePoseScoringInverse = -get_time();
  checkInversePoseEstimates(scan1, scan2, poseEstimates, minScore, parameters.maxValidationPointError,
                            parameters.useComplexValidationPointScoring, -1);
  timePoseScoringInverse += get_time();
  
  //std::cout << poseEstimates.size()<<" pose estimates left after inverse check.\n";
  
  if (parameters.doSelfSimilarityScoreAdjustment && &scan1 != &scan2)
    adjustUsingSelfSimilarityScore(scan1, scan2, poseEstimates);
  
  sortAndApplyMinScore(poseEstimates, minScore);

  //std::cout << poseEstimates.size()<<" pose estimates left after self similarity adaption.\n";
  
  timeRest += get_time();
  
  //cout << "-----\n";
  //cout << PVARN(1000*timePoseScoring);
  //cout << PVARN(1000*timeICP);
  //cout << PVARN(1000*timePoseScoring2);
  //cout << PVARN(1000*timePoseScoringInverse);
  //cout << PVARN(1000*timeRest);
  
  //poseEstimates.resize(std::min(int(poseEstimates.size()),parameters.maxNoOfRemainingPoseEstimates));
  //doRangeImageComparisonFiltering(scan1, scan2, poseEstimates, doSelfSimilarityScoreAdjustment);
  //sortAndApplyMinScore(poseEstimates, minScore);
}

void RangeImageMatching::getBestPoseEstimatesFromSortedMatches(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                                                               const pcl::PointCorrespondences6DVector& sortedFeatureMatches,
                                                               pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates, float minScore) const
{
  poseEstimates.clear();
  
  //MEASURE_FUNCTION_TIME;
  
  //double timePoseEstimates = -get_time();
  pcl::PosesFromMatches posesFromMatches;
  int maxNoOfPoseEstimatesPerScanPair = (ScanDatabaseElement::useRotationInvariance ? parameters.maxNoOfPoseEstimatesPerScanPairRotInv :
                                                                                      parameters.maxNoOfPoseEstimatesPerScanPair);
  int maxNoOfPoseEstimatesFor1Correspondence = 0.7*maxNoOfPoseEstimatesPerScanPair,
      maxNoOfPoseEstimatesFor2Correspondences = maxNoOfPoseEstimatesPerScanPair-maxNoOfPoseEstimatesFor1Correspondence;
  posesFromMatches.estimatePosesUsing1Correspondence(sortedFeatureMatches, maxNoOfPoseEstimatesFor1Correspondence, poseEstimates);
  posesFromMatches.estimatePosesUsing2Correspondences(sortedFeatureMatches, 50*maxNoOfPoseEstimatesFor2Correspondences,
                                                      maxNoOfPoseEstimatesFor2Correspondences, poseEstimates);
  //posesFromMatches.estimatePosesUsing3Correspondences(sortedFeatureMatches, 10000000, 10000, poseEstimates);
  //timePoseEstimates += get_time();
  //cout << PVARN(timePoseEstimates);
  //cout << "Found "<<poseEstimates.size()<<" possible transformations from "<<sortedFeatureMatches.size()<<" matches.\n";
  doCompleteFiltering(scan1, scan2, poseEstimates, minScore);
  //cout << "Found "<<poseEstimates.size()<<" possible transformations for minScore "<<minScore<<".\n";
}

void RangeImageMatching::adjustUsingSelfSimilarityScore(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                                                        pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates) const
{
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  //MEASURE_FUNCTION_TIME;
  
  float selfSimilarityFactor = 1.0f - 0.5f*(scan1.selfSimilarity+scan2.selfSimilarity);
  
  for (unsigned int poseEstimateIdx=0; poseEstimateIdx<poseEstimates.size(); ++poseEstimateIdx)
    poseEstimates[poseEstimateIdx].score *= selfSimilarityFactor;
}

void RangeImageMatching::checkInversePoseEstimates(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                                                   pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates,
                                                   float minScore, float maxValidationPointError, bool useComplexValidationPointScoring,
                                                   int maxNoOfValidationPoints) const
{
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  //MEASURE_FUNCTION_TIME;
  
  if (scan2.rangeImage.points.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": range image is empty.\n";
    return;
  }
  if (scan1.validationPoints.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": validation points are empty.\n";
    return;
  }
  
  if (minScore < 0.0f)
    minScore = parameters.minValidationPointScore;
  
  #pragma omp parallel for default(shared) num_threads(parameters.maxNoOfThreadsPerScanComparison) schedule(dynamic, 100)
  for (unsigned int poseEstimateIdx=0; poseEstimateIdx<poseEstimates.size(); ++poseEstimateIdx)
  {
    pcl::PosesFromMatches::PoseEstimate& poseEstimate = poseEstimates[poseEstimateIdx];
    Eigen::Isometry3f transformation;
    transformation.matrix() = poseEstimate.transformation.matrix();
    float score = getValidationPointsScore(scan2, scan1, transformation.inverse(), minScore, maxValidationPointError,
                                           useComplexValidationPointScoring, maxNoOfValidationPoints);
    poseEstimate.score = std::min(score, poseEstimate.score);
  }
}

void RangeImageMatching::getBestPoseEstimates(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                                              pcl::PointCorrespondences6DVector& featureMatches,
                                              pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates) const
{
  featureMatches.clear();
  //double timeFeatureMatching = -get_time();
  scan1.getMatches(scan2, parameters.maxDescriptorDistance, parameters.minDistanceBetweenMatches, featureMatches);
  std::sort(featureMatches.begin(), featureMatches.end(), pcl::isBetterCorrespondence);
  //timeFeatureMatching += get_time();
  //cout << PVARN(timeFeatureMatching);
  //cout << "Compared "<<scan1.features.size()<<" with "<<scan2.features.size()<<" features and found "<<featureMatches.size()<<" possible matches.\n";
  getBestPoseEstimatesFromSortedMatches(scan1, scan2, featureMatches, poseEstimates);
}

float RangeImageMatching::getSelfSimilarity(const ScanDatabaseElement& scan) const {
  //MEASURE_FUNCTION_TIME;

  float minScore = parameters.minValidationPointScore;
  //float minScore = 0.0f;
  
  pcl::PointCorrespondences6DVector tmpFeatureMatches, featureMatches;
  scan.getMatches(scan, parameters.maxDescriptorDistance, parameters.minDistanceBetweenMatches, tmpFeatureMatches);
  for (size_t matchIdx=0; matchIdx<tmpFeatureMatches.size(); ++matchIdx) {
    const pcl::PointCorrespondence6D& match = tmpFeatureMatches[matchIdx];
    if (match.index_query != match.index_match)
      featureMatches.push_back(tmpFeatureMatches[matchIdx]);
  }
  std::sort(featureMatches.begin(), featureMatches.end(), pcl::isBetterCorrespondence);
  pcl::PosesFromMatches posesFromMatches;
  pcl::PosesFromMatches::PoseEstimatesVector tmpPoseEstimates;
  posesFromMatches.estimatePosesUsing1Correspondence(featureMatches, parameters.maxNoOfPoseEstimatesPerScanPair, tmpPoseEstimates);
  
  float averageRange = 0.0f;
  int noOfPoints = 0;
  for (size_t pointIdx=0; pointIdx<scan.rangeImage.points.size(); ++pointIdx) {
    if (!scan.rangeImage.isValid(pointIdx))
      continue;
    averageRange += scan.rangeImage.getPoint(pointIdx).range;
    ++noOfPoints;
  }
  averageRange /= noOfPoints;
  //cout << PVARN(averageRange);
  
  float maxErrorTranslation = 0.5f*averageRange,
        maxErrorOrientation = deg2rad(10.0f);
  float errorTranslation, errorOrientation;
  
  pcl::PosesFromMatches::PoseEstimatesVector poseEstimates;
  for (size_t poseEstimateIdx=0; poseEstimateIdx<tmpPoseEstimates.size(); ++poseEstimateIdx) {
    Eigen::Isometry3f transformation;
    transformation.matrix() = tmpPoseEstimates[poseEstimateIdx].transformation.matrix();
    getEstimationError(transformation, Eigen::Isometry3f::Identity(), errorTranslation, errorOrientation);
    if (errorTranslation > maxErrorTranslation || errorOrientation>maxErrorOrientation)
      poseEstimates.push_back(tmpPoseEstimates[poseEstimateIdx]);
  }
  
  //doValidationPointFiltering(scan, scan, poseEstimates, minScore, parameters.maxValidationPointError);
  //sortAndApplyMinScore(poseEstimates, minScore);
  //filterSimilarPoses(poseEstimates);
  //doICP(scan, scan, poseEstimates);
  //filterSimilarPoses(poseEstimates);
  //checkInversePoseEstimates(scan, scan, poseEstimates, minScore);
  //sortAndApplyMinScore(poseEstimates, minScore);
  
  doCompleteFiltering(scan, scan, poseEstimates, minScore);
  
  for (size_t poseEstimateIdx=0; poseEstimateIdx<poseEstimates.size(); ++poseEstimateIdx) {
    Eigen::Isometry3f transformation;
    transformation.matrix() = poseEstimates[poseEstimateIdx].transformation.matrix();
    getEstimationError(transformation, Eigen::Isometry3f::Identity(), 
                       errorTranslation, errorOrientation);
    if (errorTranslation <= maxErrorTranslation && errorOrientation<=maxErrorOrientation)
      continue;
    float selfSimilarity = poseEstimates[poseEstimateIdx].score;
    //cout << "Self similarity is "<<selfSimilarity<<" ("<<PVARC(errorTranslation)<<PVARA(errorOrientation)<<").\n";
    return selfSimilarity;
  }
  return 0.0f;
}

void RangeImageMatching::getBestPoseEstimates(const ScanDatabaseElement& scan1, const ScanDatabaseElement& scan2,
                                              pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates) const
{
  pcl::PointCorrespondences6DVector featureMatches;
  getBestPoseEstimates(scan1, scan2, featureMatches, poseEstimates);
}

pcl::PosesFromMatches::PoseEstimate RangeImageMatching::getBestPoseEstimate(const ScanDatabaseElement& scan1,
                                                                            const ScanDatabaseElement& scan2) const
{
  pcl::PosesFromMatches::PoseEstimatesVector poseEstimates;
  getBestPoseEstimates(scan1, scan2, poseEstimates);
  if (!poseEstimates.empty())
    return poseEstimates[0];
  pcl::PosesFromMatches::PoseEstimate poseEstimate;
  poseEstimate.transformation = Eigen::Isometry3f::Identity();
  poseEstimate.score = 0.0f;
  return poseEstimate;
}


void RangeImageMatching::getBestPoseEstimatesFromSortedMatches(const ScanDatabaseElement& scan,
                                                               const std::vector<pcl::PointCorrespondences6DVector>& sortedFeatureMatchesPerScan,
                                                               std::vector<pcl::PosesFromMatches::PoseEstimatesVector>& poseEstimatesPerScan) const
{
  size_t noOfScans = scanDatabase.size();
  if (sortedFeatureMatchesPerScan.size() != noOfScans)
  {
    std::cerr << __PRETTY_FUNCTION__ << " - Error: sortedFeatureMatchesPerScan.size() != noOfScans\n";
    return;
  }
  poseEstimatesPerScan.clear();
  poseEstimatesPerScan.resize(noOfScans);
  #pragma omp parallel for default(shared) num_threads(parameters.maxNoOfThreadsPerDatabaseSearch) schedule(dynamic, 1)
  for (size_t scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
    getBestPoseEstimatesFromSortedMatches(*scanDatabase[scanIdx], scan, sortedFeatureMatchesPerScan[scanIdx], poseEstimatesPerScan[scanIdx]);
  }
  
  //for (size_t scanIdx=0; scanIdx<noOfScans; ++scanIdx) {
    //pcl::PointCorrespondences6DVector inverseMatches = sortedFeatureMatchesPerScan[scanIdx];
    //for (size_t matchIdx=0; matchIdx < inverseMatches.size(); ++matchIdx)  // Turn matches around - necessary to not require
    //{                                                                      // the database scans to have range images in memory
      //pcl::PointCorrespondence6D& match = inverseMatches[matchIdx];
      //std::swap(match.index1, match.index2);
      //std::swap(match.point1, match.point2);
      //match.transformation = match.transformation.inverse();
    //}
    
    //pcl::PosesFromMatches::PoseEstimatesVector& poseEstimates = poseEstimatesPerScan[scanIdx];
    //getBestPoseEstimatesFromSortedMatches(scan, *scanDatabase[scanIdx], inverseMatches, poseEstimates);
    //for (size_t poseEstimateIdx=0; poseEstimateIdx < poseEstimates.size(); ++poseEstimateIdx)  // Turn pose estimates back around
    //{
      //pcl::PosesFromMatches::PoseEstimate& poseEstimate = poseEstimates[poseEstimateIdx];
      //poseEstimate.transformation = poseEstimate.transformation.inverse();
    //}
  //}
}

void RangeImageMatching::getBestPoseEstimates(const ScanDatabaseElement& scan,
                                              std::vector<pcl::PosesFromMatches::PoseEstimatesVector>& poseEstimatesPerScan) const
{
  std::vector<pcl::PointCorrespondences6DVector> featureMatchesPerScan;
  scanDatabase.getMatches(scan, parameters.maxDescriptorDistance, parameters.minDistanceBetweenMatches,
                          featureMatchesPerScan, parameters.useKDTreeForFeatureMatching, parameters.maxDescriptorDistanceKdtree);
  for (size_t scanIdx=0; scanIdx<scanDatabase.size(); ++scanIdx)
    std::sort(featureMatchesPerScan[scanIdx].begin(), featureMatchesPerScan[scanIdx].end(), pcl::isBetterCorrespondence);
  getBestPoseEstimatesFromSortedMatches(scan, featureMatchesPerScan, poseEstimatesPerScan);
}

void RangeImageMatching::getBestPoseEstimates(const ScanDatabaseElement& scan, std::vector<int> scansToCompareWith,
                                              std::vector<pcl::PosesFromMatches::PoseEstimatesVector>& poseEstimatesPerScan,
                                              double timeLimit) const
{
  poseEstimatesPerScan.resize(scanDatabase.size());
  double startTime=get_time();
  #pragma omp parallel for default(shared) num_threads(parameters.maxNoOfThreadsPerDatabaseSearch) schedule(dynamic, 1)
  for (size_t scansToCompareWithIdx=0; scansToCompareWithIdx<scansToCompareWith.size(); ++scansToCompareWithIdx) {
    if (timeLimit > 0.0  &&  get_time()-startTime > timeLimit)
      continue;
    int scanIdx = scansToCompareWith[scansToCompareWithIdx];
    getBestPoseEstimates(*scanDatabase[scanIdx], scan, poseEstimatesPerScan[scanIdx]);
  }
}

void RangeImageMatching::getEstimationError(const Eigen::Isometry3f& estimatedPose, const Eigen::Isometry3f& truePose,
                                            float& errorTranslation, float& errorOrientation)
{
  const Eigen::Isometry3f& poseError = estimatedPose.inverse() * truePose;
  Eigen::Vector3f posError;
  float rollError, pitchError, yawError;
  pcl::getTranslationAndEulerAngles(poseError, posError[0], posError[1], posError[2], rollError, pitchError, yawError);
  errorTranslation=posError.norm();
  errorOrientation = sqrtf(powf(rollError,2)+powf(pitchError,2)+powf(yawError,2));
}

//void RangeImageMatching::recalculateAllScansInDatabase() {
  //MEASURE_FUNCTION_TIME;
//}

#pragma GCC diagnostic ignored "-Wunused-parameter"  // Do not show warnings from ROS - for some weird reason
                                                     // some stuff appears after the end of the code...
