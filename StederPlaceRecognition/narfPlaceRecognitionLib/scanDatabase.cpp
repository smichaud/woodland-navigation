#include "scanDatabase.h"

#include <iostream>
using std::cout;
using std::cerr;
#include <fstream>
#include <cmath>

#include "EXTERNALS/ais3dTools/basics/misc.h"
using Ais3dTools::deg2rad;
using Ais3dTools::rad2deg;

#pragma GCC diagnostic ignored "-Wunused-parameter"  // Do not show warnings from ROS
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#pragma GCC diagnostic warning "-Wunused-parameter"

#include "EXTERNALS/ais3dTools/basics/timeutil.h"

bool ScanDatabaseElement::useRotationInvariance = false;

struct PointWithDistance
{
  Eigen::Vector3f point;
  float distance;
  inline bool operator<(const PointWithDistance& other) const { return distance<other.distance; }
};
#include <limits>
void getUniformlyDistributedPoints(const ScanDatabaseElement::VectorOfEigenVector3f& points,
                                   int no_of_points, ScanDatabaseElement::VectorOfEigenVector3f& result)
{
  result.clear();
  no_of_points = std::min(no_of_points, (int)points.size());
  if (no_of_points==0)
    return;
  
  std::vector<PointWithDistance> remaining_points;
  remaining_points.resize(points.size());
  for (unsigned int i=0; i<points.size(); ++i) {
    remaining_points[i].point = points[i];
    remaining_points[i].distance = std::numeric_limits<float>::max();
  }
  
  while (true)
  {
    result.push_back(remaining_points.back().point);
    remaining_points.pop_back();
    for (unsigned int point_idx=0; point_idx<remaining_points.size(); ++point_idx)
    {
      PointWithDistance& point = remaining_points[point_idx];
      point.distance = std::min(point.distance, (result.back()-point.point).squaredNorm());
    }
    if ((int)result.size() >= no_of_points)
      break;
    std::sort(remaining_points.begin(), remaining_points.end());
  }
}
#include <algorithm>
template <typename ContainerType>
void freeContainerMemory(ContainerType& t) {
  ContainerType tmp;
  std::swap(t, tmp);
}

////////////////////////////////////////////
ScanDatabaseElement::ScanDatabaseElement()
{
  reset();
}

////////////////////////////////////////////
ScanDatabaseElement::ScanDatabaseElement(const ScanDatabaseElement& other)
{
  deepCopy(other);
}

////////////////////////////////////////////
const ScanDatabaseElement& ScanDatabaseElement::operator=(const ScanDatabaseElement& other)
{
  deepCopy(other);
  return *this;
}

////////////////////////////////////////////
ScanDatabaseElement::~ScanDatabaseElement()
{
  reset();
}

////////////////////////////////////////////
void ScanDatabaseElement::deepCopy(const ScanDatabaseElement& other)
{
  reset();
  pointCloudFileName = other.pointCloudFileName;
  pointCloudData = other.pointCloudData;
  pointCloudDataFarRanges = other.pointCloudDataFarRanges;
  pointCloud = other.pointCloud;
  pointCloudFarRanges = other.pointCloudFarRanges;
  rangeImage = other.rangeImage;
  features.resize(other.features.size());
  for (size_t featureIdx=0; featureIdx<features.size(); ++featureIdx)
    features[featureIdx] = new pcl::Narf(*other.features[featureIdx]);
  featuresForDictionary.resize(other.featuresForDictionary.size());
  for (size_t featureIdx=0; featureIdx<featuresForDictionary.size(); ++featureIdx)
    featuresForDictionary[featureIdx] = new pcl::Narf(*other.featuresForDictionary[featureIdx]);
  validationPoints = other.validationPoints;
  selfSimilarity = other.selfSimilarity;
  knownPoses = other.knownPoses;
  dictionaryHistogram = other.dictionaryHistogram;
  narfsForCompleteScan.resize(other.narfsForCompleteScan.size());
  for (size_t featureIdx=0; featureIdx<narfsForCompleteScan.size(); ++featureIdx)
    narfsForCompleteScan[featureIdx] = new pcl::Narf(*other.narfsForCompleteScan[featureIdx]);
  //kdTreeForRangeImage = other.kdTreeForRangeImage;
}

////////////////////////////////////////////
void ScanDatabaseElement::reset()
{
  pointCloudFileName = "";
  resetClouds();
  resetRangeImage();
  resetFeatures();
  resetDictionaryFeatures();
  resetGlobalFeatures();
  resetValidationPoints();
  selfSimilarity = 0.0f;
  knownPoses.clear();
  //kdTreeForRangeImage.clear();
}
void ScanDatabaseElement::resetFeatures() {
  for (unsigned int i=0; i<features.size(); ++i)
    delete features[i];
  freeContainerMemory(features);
}
void ScanDatabaseElement::resetDictionaryFeatures() {
  for (unsigned int i=0; i<featuresForDictionary.size(); ++i)
    delete featuresForDictionary[i];
  freeContainerMemory(featuresForDictionary);
}
void ScanDatabaseElement::resetGlobalFeatures() {
  while (!narfsForCompleteScan.empty()) {
    delete narfsForCompleteScan.back();
    narfsForCompleteScan.pop_back();
  }
}

void ScanDatabaseElement::resetValidationPoints()
{
  freeContainerMemory(validationPoints);
}
void ScanDatabaseElement::resetClouds()
{
  // This is all a little weird, but we want to make sure that the memory inside of the vectors is freed
  pointCloudData = pointCloudDataFarRanges = pcl::PCLPointCloud2();
  freeContainerMemory(pointCloudData.data);
  freeContainerMemory(pointCloudDataFarRanges.data);
  pointCloud = pointCloudFarRanges = PointCloudType();
  freeContainerMemory(pointCloud.points);
  freeContainerMemory(pointCloudFarRanges.points);
}
void ScanDatabaseElement::resetRangeImage()
{
  // This is all a little weird, but we want to make sure that the memory inside of the vectors is freed
  rangeImage = pcl::RangeImage();
  freeContainerMemory(rangeImage.points);
}

struct NullDeleter {
  void operator()(void const *) const {}
};
boost::shared_ptr<pcl::RangeImage> ScanDatabaseElement::getRangeImageAsBoostSharedPtr() {
  return boost::shared_ptr<pcl::RangeImage>(&rangeImage, NullDeleter());
}

//void ScanDatabaseElement::updateKdTree() {
  ////MEASURE_FUNCTION_TIME;
  //kdTreeForRangeImage.setInputCloud(getRangeImageAsBoostSharedPtr());
  
  ////kdTreeForRangeImage.clear();
  ////for (size_t pointIdx=0; pointIdx<rangeImage.points.size(); ++pointIdx)
    ////if (rangeImage.isValid(pointIdx))
      ////kdTreeForRangeImage.insert(rangeImage.points[pointIdx].getVector3fMap());
  ////kdTreeForRangeImage.optimise();
//}

void ScanDatabaseElement::loadPointCloud()
{
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";
  Eigen::Vector4f sensorPos;  Eigen::Quaternionf sensorOrientation;
  if (pcl::io::loadPCDFile(pointCloudFileName, pointCloudData, sensorPos, sensorOrientation) == -1)
  {
    cerr << "Could not open file \""<<pointCloudFileName<<"\".\n";
    return;
  }
  //if (pcl::getFieldIndex(pointCloudData, "vp_x") < 0)
  //{
    ////cout << "No per point viewpoint information available in cloud.\n";
    //pcl::PointCloud<pcl::PointXYZ> tmpCloud;
    //fromPCLPointCloud2(pointCloudData, tmpCloud);
    //pcl::PointWithViewpoint tmpPoint;
    //tmpPoint.vp_x=tmpPoint.vp_y=tmpPoint.vp_z = 0.0f;
    //for (int i=0; i<int(tmpCloud.points.size()); ++i)
    //{
      //tmpPoint.getVector4fMap() = tmpCloud.points[i].getVector4fMap();
      //pointCloud.points.push_back(tmpPoint);
    //}
  //}
  //else
  fromPCLPointCloud2(pointCloudData, pointCloud);
  pointCloud.sensor_origin_ = sensorPos;  pointCloud.sensor_orientation_ = sensorOrientation;
  
  std::string fileNameFarRanges = pcl::getFilenameWithoutExtension(pointCloudFileName)
                                  + "_far_ranges."+pcl::getFileExtension(pointCloudFileName);

  std::ifstream testFile(fileNameFarRanges.c_str());
  bool fileDoesNotExists = !testFile;
  testFile.close();
  
  if (fileDoesNotExists || pcl::io::loadPCDFile(fileNameFarRanges, pointCloudDataFarRanges, sensorPos, sensorOrientation) == -1)
  {
    //cerr << "No file for far ranges found - looking for them in original file.\n";
    //pcl::RangeImage::extractFarRanges(pointCloudData, pointCloudFarRanges);
    //toROSMsg(pointCloudFarRanges, pointCloudDataFarRanges);
  }
  else
  {
    fromPCLPointCloud2(pointCloudDataFarRanges, pointCloudFarRanges);
    pointCloudFarRanges.sensor_origin_ = sensorPos;  pointCloudFarRanges.sensor_orientation_ = sensorOrientation;
  }
}

template <typename PointCloudType>
void integrateFarRanges (pcl::RangeImage& rangeImage, const PointCloudType& far_ranges)
{
  float x_real, y_real, range_of_current_point;
  for (typename PointCloudType::const_iterator it = far_ranges.points.begin (); it != far_ranges.points.end (); ++it)
  {
    //if (!isFinite (*it))  // Check for NAN etc
      //continue;
    Eigen::Vector3f current_point = it->getVector3fMap ();
    
    rangeImage.getImagePoint (current_point, x_real, y_real, range_of_current_point);
    
    int floor_x = static_cast<int> (lrintf (floor (x_real))), 
        floor_y = static_cast<int> (lrintf (floor (y_real))),
        ceil_x  = static_cast<int> (lrintf (ceil (x_real))),
        ceil_y  = static_cast<int> (lrintf (ceil (y_real)));
    
    int neighbor_x[4], neighbor_y[4];
    neighbor_x[0]=floor_x; neighbor_y[0]=floor_y;
    neighbor_x[1]=floor_x; neighbor_y[1]=ceil_y;
    neighbor_x[2]=ceil_x;  neighbor_y[2]=floor_y;
    neighbor_x[3]=ceil_x;  neighbor_y[3]=ceil_y;
    
    for (int i=0; i<4; ++i)
    {
      int x=neighbor_x[i], y=neighbor_y[i];
      if (!rangeImage.isInImage (x, y))
        continue;
      pcl::PointWithRange& image_point = rangeImage.getPoint (x, y);
      if (!pcl_isfinite (image_point.range))
        image_point.range = std::numeric_limits<float>::infinity ();
    }
  }
}

void ScanDatabaseElement::createRangeImage(float angularResolution, float noiseLevel,
                                           pcl::RangeImage::CoordinateFrame coordinateFrame, float maximumRange)
{
  //cout << __PRETTY_FUNCTION__ << " called.\n";
  if (pointCloud.points.empty())
  {
    rangeImage.reset();
    return;
  }
  
  Eigen::Isometry3f sensorPose = (Eigen::Isometry3f)Eigen::Translation3f(
                                  pointCloud.sensor_origin_[0], pointCloud.sensor_origin_[1], pointCloud.sensor_origin_[2]) *
                                pointCloud.sensor_orientation_;
  //cout << "Creating a range image from a point cloud of size "<<pointCloud.points.size()
       //<< " with angular resolution "<<rad2deg(angularResolution)<<"deg"
       //<< ", noise level "<<noiseLevel<<"m"
       //<< ", and sensor pos "<<sensorPose.translation()[0]<<", "<<sensorPose.translation()[1]<<", "<<sensorPose.translation()[2]
       //<< ".\n";

  //// HACK to remove points in sensor origin
  //for (size_t y=0; y<pointCloud.height; ++y) {
    //for (size_t x=0; x<pointCloud.width; ++x) {
      ////std::cout << x<<","<<y<<": "
                ////<< pointCloud.points[y*pointCloud.width+x].x<<","
                ////<< pointCloud.points[y*pointCloud.width+x].y<<","
                ////<< pointCloud.points[y*pointCloud.width+x].z
                ////<<": "<<(pcl::isFinite(pointCloud.points[y*pointCloud.width+x])?"":"not ")<<"finite\n";
      //if (pointCloud.points[y*pointCloud.width+x].getVector3fMap().squaredNorm()<1e-6) {
        //pointCloud.points[y*pointCloud.width+x].x =
        //pointCloud.points[y*pointCloud.width+x].y =
        //pointCloud.points[y*pointCloud.width+x].z = std::numeric_limits<float>::quiet_NaN();
      //}
    //}
  //}
  rangeImage.createFromPointCloud(pointCloud, angularResolution, deg2rad(360.0f), deg2rad(180.0f),
                                  sensorPose, coordinateFrame, noiseLevel, 0.0f, 0);
  if (!pointCloudFarRanges.points.empty())
    integrateFarRanges(rangeImage, pointCloudFarRanges);
  else {
    std::cout << "No far range information available -> Setting unseen areas to far ranges.\n";
    rangeImage.setUnseenToMaxRange();
  }
  //cout << PVARN(rangeImage);
  if (maximumRange > 0.0f) {
    //cout << PVARN(maximumRange);
    for (int point_idx=0; point_idx<int(rangeImage.points.size()); ++point_idx)
      if (rangeImage.points[point_idx].range > maximumRange)
        rangeImage.points[point_idx].range = std::numeric_limits<float>::infinity();
  }
}

//void ScanDatabaseElement::extractNARFs(float supportSize, int descriptorSize, bool rotationInvariant,
                                       //pcl::NarfKeypoint& keyPointDetector)
//{
  //resetFeatures();
  //keyPointDetector.setRangeImage(&rangeImage);
  //const pcl::PointCloud<pcl::InterestPoint>& keyPoints = keyPointDetector.getInterestPoints();
  //pcl::Narf::extractForInterestPoints(rangeImage, keyPoints, descriptorSize,
                                      //supportSize, rotationInvariant, features);
  //for (size_t i=0; i<features.size(); ++i) {
    //features[i]->freeSurfacePatch();
    ////cout << PVARN(features[i]->getTransformation().matrix());
  //}
  //keyPointDetector.setRangeImage(NULL);
//}

void ScanDatabaseElement::extractNARFs(float supportSize, int descriptorSize, bool rotationInvariant,
                                       MyPcl::NarfssKeypoint& keyPointDetector, bool doNotResetKeypointDetector)
{
  resetFeatures();
  keyPointDetector.setRangeImage(&rangeImage);
  const pcl::PointCloud<pcl::InterestPoint>& keyPoints = keyPointDetector.getKeypoints();
  pcl::Narf::extractForInterestPoints(rangeImage, keyPoints, descriptorSize,
                                      supportSize, rotationInvariant, features);
  for (size_t i=0; i<features.size(); ++i) {
    features[i]->freeSurfacePatch();
    //cout << PVARN(features[i]->getTransformation().matrix());
  }
  if (!doNotResetKeypointDetector)
    keyPointDetector.setRangeImage(NULL);
}

void ScanDatabaseElement::extractDictionaryNARFs(float supportSize, int descriptorSize, bool rotationInvariant,
                                                 MyPcl::NarfssKeypoint& keyPointDetector)
{
  resetDictionaryFeatures();
  //double time1 = Ais3dTools::getTime();
  keyPointDetector.setRangeImage(&rangeImage);
  //double time2 = Ais3dTools::getTime();
  const pcl::PointCloud<pcl::InterestPoint>& keyPoints = keyPointDetector.getKeypoints();
  //double time3 = Ais3dTools::getTime();
  pcl::Narf::extractForInterestPoints(rangeImage, keyPoints, descriptorSize,
                                      supportSize, rotationInvariant, featuresForDictionary);
  //double time4 = Ais3dTools::getTime();
  for (size_t i=0; i<featuresForDictionary.size(); ++i) {
    featuresForDictionary[i]->freeSurfacePatch();
  }
  //double time5 = Ais3dTools::getTime();
  keyPointDetector.setRangeImage(NULL);
  //cout << PVARN(featuresForDictionary.size());
  //cout << "PR: "<<PVARC(time2-time1) << PVARC(time3-time2) << PVARC(time4-time3) << PVARN(time5-time4);
}


void ScanDatabaseElement::saveRangeImage() const
{
  if (pointCloudFileName.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": No file name.\n";
    return;
  }
  if (rangeImage.points.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": Range image is empty.\n";
    return;
  }

  std::string filename = pcl::getFilenameWithoutExtension(pointCloudFileName)+"_RangeImage.dat";
  std::ofstream file(filename.c_str());
  file.write(reinterpret_cast<const char*>(&rangeImage.width), sizeof(&rangeImage.width));
  file.write(reinterpret_cast<const char*>(&rangeImage.height), sizeof(&rangeImage.height));
  file.write(reinterpret_cast<const char*>(&rangeImage.is_dense), sizeof(&rangeImage.is_dense));
  file.write(reinterpret_cast<const char*>(rangeImage.getTransformationToRangeImageSystem().data()), 16*sizeof(float));
  float angular_resolution = rangeImage.getAngularResolution();
  file.write(reinterpret_cast<const char*>(&angular_resolution), sizeof(angular_resolution));
  int image_offset_x=rangeImage.getImageOffsetX(), image_offset_y=rangeImage.getImageOffsetY();
  file.write(reinterpret_cast<const char*>(&image_offset_x), sizeof(image_offset_x));
  file.write(reinterpret_cast<const char*>(&image_offset_y), sizeof(image_offset_y));
  for (unsigned int i=0; i<rangeImage.points.size(); ++i) {
    const pcl::PointWithRange& point = rangeImage.points[i];
    file.write(reinterpret_cast<const char*>(&point.x), sizeof(point.x));
    file.write(reinterpret_cast<const char*>(&point.y), sizeof(point.y));
    file.write(reinterpret_cast<const char*>(&point.z), sizeof(point.z));
    file.write(reinterpret_cast<const char*>(&point.range), sizeof(point.range));
  }
  file.close();
}

void ScanDatabaseElement::loadRangeImage()
{
  if (pointCloudFileName.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": No file name.\n";
    return;
  }
  std::string filename = pcl::getFilenameWithoutExtension(pointCloudFileName)+"_RangeImage.dat";
  std::ifstream file(filename.c_str());
  
  resetRangeImage();
  file.read(reinterpret_cast<char*>(&rangeImage.width), sizeof(&rangeImage.width));
  file.read(reinterpret_cast<char*>(&rangeImage.height), sizeof(&rangeImage.height));
  file.read(reinterpret_cast<char*>(&rangeImage.is_dense), sizeof(&rangeImage.is_dense));
  Eigen::Isometry3f to_range_image_system;
  file.read(reinterpret_cast<char*>(to_range_image_system.data()), 16*sizeof(float));
  rangeImage.setTransformationToRangeImageSystem(to_range_image_system);
  float angular_resolution;
  file.read(reinterpret_cast<char*>(&angular_resolution), sizeof(angular_resolution));
  rangeImage.setAngularResolution(angular_resolution);
  int image_offset_x, image_offset_y;
  file.read(reinterpret_cast<char*>(&image_offset_x), sizeof(image_offset_x));
  file.read(reinterpret_cast<char*>(&image_offset_y), sizeof(image_offset_y));
  rangeImage.setImageOffsets(image_offset_x, image_offset_y);
  int rangeImageSize = rangeImage.width*rangeImage.height;
  rangeImage.points.resize(rangeImageSize);
  for (int i=0; i<rangeImageSize; ++i) {
    pcl::PointWithRange& point = rangeImage.points[i];
    file.read(reinterpret_cast<char*>(&point.x), sizeof(point.x));
    file.read(reinterpret_cast<char*>(&point.y), sizeof(point.y));
    file.read(reinterpret_cast<char*>(&point.z), sizeof(point.z));
    file.read(reinterpret_cast<char*>(&point.range), sizeof(point.range));
  }
  file.close();
}


void ScanDatabaseElement::saveNARFs() const {
  if (pointCloudFileName.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": No file name.\n";
    return;
  }
  if (features.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": Features list is empty.\n";
  }
  else {
    std::string rotInv = (useRotationInvariance ? "RotInv" : "");
    std::string filename = pcl::getFilenameWithoutExtension(pointCloudFileName)+"_NARFs"+rotInv+".dat";
    std::ofstream file(filename.c_str());
    uint32_t noOfFeatures = features.size();
    file.write(reinterpret_cast<const char*>(&noOfFeatures), sizeof(noOfFeatures));
    for (unsigned int featureIdx=0; featureIdx<features.size(); ++featureIdx)
      features[featureIdx]->saveBinary(file);
    file.close();
  }
}

void ScanDatabaseElement::saveDictionaryNARFs() const {
  if (pointCloudFileName.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": No file name.\n";
    return;
  }
  if (featuresForDictionary.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": Feature list for dictionary is empty.\n";
  }
  else {
    std::string rotInv = (useRotationInvariance ? "RotInv" : "");
    std::string filename = pcl::getFilenameWithoutExtension(pointCloudFileName)+"_NARFs"+rotInv+"_dict.dat";
    std::ofstream file(filename.c_str());
    uint32_t noOfFeatures = featuresForDictionary.size();
    file.write(reinterpret_cast<const char*>(&noOfFeatures), sizeof(noOfFeatures));
    for (unsigned int featureIdx=0; featureIdx<featuresForDictionary.size(); ++featureIdx)
      featuresForDictionary[featureIdx]->saveBinary(file);
    file.close();
  }
}

void ScanDatabaseElement::saveDictionaryHistogram() const {
  if (pointCloudFileName.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": No file name.\n";
    return;
  }
  std::string rotInv = (useRotationInvariance ? "RotInv" : "");
  std::string filename = pcl::getFilenameWithoutExtension(pointCloudFileName)+"_histogram"+rotInv+".dat";
  std::ofstream file(filename.c_str());
  file << dictionaryHistogram.size();
  for (size_t histogramIdx=0; histogramIdx<dictionaryHistogram.size(); ++histogramIdx)
    file << " " << dictionaryHistogram[histogramIdx];
  file << "\n";
  file.close();
}

void ScanDatabaseElement::loadDictionaryHistogram() {
  dictionaryHistogram.clear();
  if (pointCloudFileName.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": No file name.\n";
    return;
  }
  std::string rotInv = (useRotationInvariance ? "RotInv" : "");
  std::string filename = pcl::getFilenameWithoutExtension(pointCloudFileName)+"_histogram"+rotInv+".dat";
  std::ifstream file(filename.c_str());
  if (!file)
    return;
  int histogramSize;
  file >> histogramSize;
  dictionaryHistogram.resize(histogramSize);
  for (size_t histogramIdx=0; histogramIdx<dictionaryHistogram.size(); ++histogramIdx)
    file >> dictionaryHistogram[histogramIdx];
  file.close();
}

void ScanDatabaseElement::loadNARFs() {
  resetFeatures();
  if (pointCloudFileName.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": No file name.\n";
    return;
  }
  std::string rotInv = (useRotationInvariance ? "RotInv" : "");
  std::string filename = pcl::getFilenameWithoutExtension(pointCloudFileName)+"_NARFs"+rotInv+".dat";
  std::ifstream file(filename.c_str());
  if (!file) {
    std::cerr << __PRETTY_FUNCTION__ << ": Could not open file \""<<filename<<"\".\n";
    return;
  }
  uint32_t noOfFeatures;
  file.read(reinterpret_cast<char*>(&noOfFeatures), sizeof(noOfFeatures));
  for (unsigned int featureIdx=0; featureIdx<noOfFeatures; ++featureIdx)
  {
    features.push_back(new pcl::Narf);
    features.back()->loadBinary(file);
  }
  file.close();
}
  
void ScanDatabaseElement::loadDictionaryNARFs() {
  resetDictionaryFeatures();
  std::string rotInv = (useRotationInvariance ? "RotInv" : "");
  std::string filename = pcl::getFilenameWithoutExtension(pointCloudFileName)+"_NARFs"+rotInv+"_dict.dat";
  //std::string filename = pcl::getFilenameWithoutExtension(pointCloudFileName)+"_NARFs"+rotInv+".dat";
  std::ifstream file(filename.c_str());
  if (!file) {
    //std::cerr << __PRETTY_FUNCTION__ << ": Could not open file \""<<filename<<"\".\n";
    return;
  }
  uint32_t noOfFeatures;
  file.read(reinterpret_cast<char*>(&noOfFeatures), sizeof(noOfFeatures));
  for (unsigned int featureIdx=0; featureIdx<noOfFeatures; ++featureIdx)
  {
    featuresForDictionary.push_back(new pcl::Narf);
    featuresForDictionary.back()->loadBinary(file);
  }
  file.close();
}

void ScanDatabaseElement::saveValidationPoints() const
{
  if (pointCloudFileName.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": No file name.\n";
    return;
  }
  if (validationPoints.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": Validation point list is empty.\n";
    return;
  }
  
  std::string filename = pcl::getFilenameWithoutExtension(pointCloudFileName)+"_ValidationPoints.dat";
  std::ofstream file(filename.c_str());
  uint32_t noOfValidationPoints = validationPoints.size();
  file.write(reinterpret_cast<const char*>(&noOfValidationPoints), sizeof(noOfValidationPoints));
  for (unsigned int validationPointIdx=0; validationPointIdx<noOfValidationPoints; ++validationPointIdx)
  {
    const Eigen::Vector3f& validationPoint = validationPoints[validationPointIdx];
    file.write(reinterpret_cast<const char*>(validationPoint.data()), 3*sizeof(validationPoint[0]));
  }
  file.close();
}


void ScanDatabaseElement::loadValidationPoints()
{
  resetValidationPoints();
  if (pointCloudFileName.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": No file name.\n";
    return;
  }
  std::string filename = pcl::getFilenameWithoutExtension(pointCloudFileName)+"_ValidationPoints.dat";
  std::ifstream file(filename.c_str());
  if (!file)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Could not open file \""<<filename<<"\".\n";
    return;
  }
  uint32_t noOfValidationPoints;
  file.read(reinterpret_cast<char*>(&noOfValidationPoints), sizeof(noOfValidationPoints));
  validationPoints.resize(noOfValidationPoints);
  for (unsigned int validationPointIdx=0; validationPointIdx<noOfValidationPoints; ++validationPointIdx)
  {
    Eigen::Vector3f& validationPoint = validationPoints[validationPointIdx];
    file.read(reinterpret_cast<char*>(validationPoint.data()), 3*sizeof(validationPoint[0]));
  }
  file.close();
  
  calculateNormalsForValidationPoints();
}

void ScanDatabaseElement::loadInfoFile() {
  knownPoses.clear();
  if (pointCloudFileName.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": No file name.\n";
    return;
  }
  std::string filename = pcl::getFilenameWithoutExtension(pointCloudFileName)+"_info.dat";
  std::ifstream file(filename.c_str());
  if (!file)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Could not open file \""<<filename<<"\".\n";
    return;
  }
  
  while (!file.eof()) {
    std::string tmpString;
    std::getline(file, tmpString);
    if (tmpString.empty() || tmpString[0]=='#')
      continue;
    std::stringstream line(tmpString);
    
    std::string token;
    line >> token;
    //cout << PVARN(token);
    if (token == "selfSimilarity") {
      line >> selfSimilarity;
      continue;
    }
    else if (token=="IncrementalScanMatching:" || token=="Odometry:" || token=="SLAM:") {
      float x, y, z, roll, pitch, yaw;
      line >> x >> y >> z >> roll >> pitch >> yaw;
      Eigen::Isometry3f pose;
      pose.matrix() = Ais3dTools::TransformationRepresentation::getMatrixFromTranslationAndEuler(x, y, z, roll, pitch, yaw);
      token = token.substr(0,token.size()-1);
      knownPoses[token] = pose;
    }
    else {
      std::string unknownInfoFileElement;
      std::getline(line, unknownInfoFileElement);
      //cout << "Read unknown info file element of type: \""<<token<<unknownInfoFileElement<<"\".\n";
      unknownInfoFileElements[token] = unknownInfoFileElement;
    }
  }
  file.close();
  
  ////HACK to change coordinate system
  //std::stringstream ss;
  //while (true) {
    //std::string token;
    //if (file.peek()=='#')
    //{
      //std::getline(file, token);
      //ss << token<<"\n";
      //continue;
    //}
    //file >> token;
    //if (token.empty() || file.eof())
      //break;
    //float x, y, z, roll, pitch, yaw;
    //file >> x >> y >> z >> roll >> pitch >> yaw;
    //Eigen::Isometry3f pose;
    //pcl::getTransformation(x, y, z, roll, pitch, yaw, pose);

    //Eigen::Isometry3f coord;
    //coord(0,0)= 0.0f; coord(0,1)= 0.0f; coord(0,2)=1.0f; coord(0,3)=0.0f;
    //coord(1,0)=-1.0f; coord(1,1)= 0.0f; coord(1,2)=0.0f; coord(1,3)=0.0f;
    //coord(2,0)= 0.0f; coord(2,1)=-1.0f; coord(2,2)=0.0f; coord(2,3)=0.0f;
    //coord(3,0)= 0.0f; coord(3,1)= 0.0f; coord(3,2)=0.0f; coord(3,3)=1.0f;
    //pose = pose * coord;
    //pcl::getTranslationAndEulerAngles(pose, x, y, z, roll, pitch, yaw);
    //ss << token<<" "<<x<<" "<<y<<" "<<z<<" "<<roll<<" "<<pitch<<" "<<yaw<<"\n";
    //token = token.substr(0,token.size()-1);
    //knownPoses[token] = pose;
  //}
  //file.close();
  //std::ofstream file2(filename.c_str());
  //file2 << ss.str();
  //cout << ss.str();
  //file2.close();
}

void ScanDatabaseElement::saveInfoFile() const
{
  if (pointCloudFileName.empty()) {
    std::cerr << __PRETTY_FUNCTION__ << ": No file name.\n";
    return;
  }
  
  std::string filename = pcl::getFilenameWithoutExtension(pointCloudFileName)+"_info.dat";
  std::ofstream file(filename.c_str());
  file << "# Known values for this scan\n";
  file << "selfSimilarity "<<selfSimilarity<<"\n";
  for (std::map<std::string, std::string>::const_iterator it=unknownInfoFileElements.begin(); it!=unknownInfoFileElements.end(); ++it) {
    file << it->first<<it->second<<"\n";
  }
  file << "# Known 6DOF positions for this scan (format: x y z roll pitch yaw)\n";
  for (MapStringToIsometry3f::const_iterator it=knownPoses.begin(); it!=knownPoses.end(); ++it) {
    //if (it->first != "Odometry")
      //continue;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(it->second, x, y, z, roll, pitch, yaw);
    file << it->first<<": "<<x<<" "<<y<<" "<<z<<" "<<roll<<" "<<pitch<<" "<<yaw<<"\n";
  }
  file.close();
}

void integrateIntoMatchList(const pcl::PointCorrespondence6D& match,
                            pcl::PointCorrespondences6DVector& featureMatches,
                            float minDistanceBetweenMatchesSquared)
{
  // We will now check, if there is already a match to the same scene feature that is close to the current view feature (in 3D space)
  // If this is the case we will only keep the better match
  int replaceIdx = -1;
  for (int matchIdx=featureMatches.size()-1; matchIdx>=0; --matchIdx)
  {
    pcl::PointCorrespondence6D& alreadyExistingMatch = featureMatches[matchIdx];
    if (alreadyExistingMatch.index_query != match.index_query)  // Since we are just adding the matches for this feature they are all at the end of the list
      break;
    
    if ((match.point2-alreadyExistingMatch.point2).squaredNorm() <= minDistanceBetweenMatchesSquared)
    {
      if (match.weight <= alreadyExistingMatch.weight)
        return;
      else
      {
        replaceIdx = matchIdx;
        break;
      }
    }
  }
  
  if (replaceIdx < 0)
    featureMatches.push_back(match);
  else
    featureMatches[replaceIdx] = match;
}

void ScanDatabaseElement::getMatches(const pcl::Narf& feature, int featureId, float maxDescriptorDistance,
                                     float minDistanceBetweenMatches, pcl::PointCorrespondences6DVector& featureMatches) const
{
  float minDistanceBetweenMatchesSquared = minDistanceBetweenMatches*minDistanceBetweenMatches;
  pcl::PointCorrespondence6D match;
  match.index_match = featureId;
  match.point2 = feature.getPosition();
  for (unsigned int featureIdx2=0; featureIdx2<features.size(); ++featureIdx2)
  {
    const pcl::Narf& feature2 = *features[featureIdx2];
    float descriptor_distance = feature.getDescriptorDistance(feature2);
    if (descriptor_distance > maxDescriptorDistance)
      continue;
    match.weight  = 1.0f-descriptor_distance;
    //if (match.score < 0 || match.score > 1)
      //std::cerr << PVARN(descriptor_distance);
    match.index_query = featureIdx2;
    match.point1 = feature2.getPosition();

    match.transformation = feature2.getTransformation().inverse() * feature.getTransformation();
    //std::cout << PVARC(match.transformation*match.point2)<<PVARN(match.point1);
    
    integrateIntoMatchList(match, featureMatches, minDistanceBetweenMatchesSquared);
  }
}

void ScanDatabaseElement::getMatches(const ScanDatabaseElement& scan, float maxDescriptorDistance,
                                     float minDistanceBetweenMatches, pcl::PointCorrespondences6DVector& featureMatches) const
{
  for (unsigned int featureIdx=0; featureIdx<scan.features.size(); ++featureIdx)
    getMatches(*scan.features[featureIdx], featureIdx, maxDescriptorDistance,
               minDistanceBetweenMatches, featureMatches);
}


struct IndexWithValue {
  int index;
  float value;
  float distance;
  bool erased;
};
inline bool hasHigherValue(const IndexWithValue& x1, const IndexWithValue& x2) {
  return (x1.value > x2.value);
}

void ScanDatabaseElement::extractValidationPoints(int noOfPoints, MyPcl::NarfssKeypoint& keyPointDetector, bool reuseKeypointDetector)
{
  //MEASURE_FUNCTION_TIME;
  resetValidationPoints();
  
  std::vector<IndexWithValue> points;
  
  if (!reuseKeypointDetector)
    keyPointDetector.setRangeImage(&rangeImage);
  const std::vector<float>& interestImage = keyPointDetector.getInterestImage();

  //const std::vector<const pcl::RangeImage*>& getRangeImageScaleSpace() const { return range_image_scale_space_; }
  //const std::vector<std::vector<float>* >& getInterestImageScaleSpace() const { return interest_image_scale_space_; }
  
  int reductionRadius = std::max(0, int(lrint(log2(0.01f*(float(rangeImage.points.size())/float(noOfPoints))))));
  //std::cout << PVARN(reductionRadius);
  
  //double startTime = get_time();
  //for (size_t point_idx=0; point_idx<interestImage.size(); ++point_idx) {
  for (int y=reductionRadius; y<int(rangeImage.height)-reductionRadius; y+=reductionRadius+1) {
    for (int x=reductionRadius; x<int(rangeImage.width)-reductionRadius; x+=reductionRadius+1) {
      //std::cout << PVARC(x)<<PVARN(y);
      IndexWithValue iwv;
      iwv.value = 0.0f;
      iwv.distance = std::numeric_limits<float>::max();
      iwv.erased = false;
      for (int y2=y-reductionRadius; y2<=y+reductionRadius; ++y2) {
        for (int x2=x-reductionRadius; x2<=x+reductionRadius; ++x2) {
          int index2 = y2*rangeImage.width + x2;
          if (!rangeImage.isValid(index2))
            continue;
          float interestValue = interestImage[index2];
          if (interestValue <= iwv.value)
            continue;
          iwv.value = interestValue;
          iwv.index = index2;
        }
      }
      if (iwv.value > 1e-4)
        points.push_back(iwv);
    }
  }
  keyPointDetector.setRangeImage(NULL);
  
  std::sort(points.begin(), points.end(), hasHigherValue);
  
  //points.resize(noOfPoints*10);
  
  //std::cout << PVARN(points.size());
  
  if (points.empty())
    return;
  
  validationPoints.push_back(rangeImage.points[points[0].index].getVector3fMap());
  
  points[0].erased = true;
  float minDistanceBetweenPointsSquared = 0.0f;
  for (std::vector<IndexWithValue>::const_iterator it=points.begin(); it!=points.end(); ++it) {
    Eigen::Vector3f point = rangeImage.points[it->index].getVector3fMap();
    minDistanceBetweenPointsSquared = std::max(minDistanceBetweenPointsSquared, (point-validationPoints.back()).squaredNorm());
  }
  minDistanceBetweenPointsSquared *= 0.25f;  // Squared value needs 1/4 to have half the distance
  
  while(int(validationPoints.size())<noOfPoints) {
    int nextPointIdx = -1;
    for (std::vector<IndexWithValue>::iterator it=points.begin(); it!=points.end(); ++it) {
      IndexWithValue& iwv = *it;
      if (iwv.erased)
        continue;
      Eigen::Vector3f point = rangeImage.points[iwv.index].getVector3fMap();
      iwv.distance = std::min(iwv.distance, (point-validationPoints.back()).squaredNorm());
      if (iwv.distance < minDistanceBetweenPointsSquared || nextPointIdx>=0)
        continue;
      nextPointIdx = iwv.index;
      //std::cout << PVARN(iwv.value);
      it->erased = true;
    }
    if (nextPointIdx<0) {
      if (validationPoints.size() == points.size())
        break;
      minDistanceBetweenPointsSquared *= 0.25f;
      //std::cout << PVARN(validationPoints.size());
      //std::cout << PVARN(sqrtf(minDistanceBetweenPointsSquared));
      continue;
    }
    validationPoints.push_back(rangeImage.points[nextPointIdx].getVector3fMap());
  }

  calculateNormalsForValidationPoints();
  
  //for (size_t validationPointIdx=0; validationPointIdx<validationPoints.size(); ++validationPointIdx) {
    //const Eigen::Vector3f& p = validationPoints[validationPointIdx];
    //if (!pcl_isfinite(p.x()) || !pcl_isfinite(p.y()) || !pcl_isfinite(p.z()))
      //std::cerr << "\n\nAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH\n\n";
  //}
  
  //std::cout<<"Took "<<get_time()-startTime<<"s.\n";
  //std::cout << PVARN(validationPoints.size());
}

void ScanDatabaseElement::calculateNormalsForValidationPoints() {
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";

  validationPoints.normals.clear();
  validationPoints.normals.resize(validationPoints.size(), Eigen::Vector3f(1,0,0));
  for (size_t pointIdx=0; pointIdx<validationPoints.size(); ++pointIdx) {
    int x, y;
    rangeImage.getImagePoint(validationPoints[pointIdx], x, y);
    rangeImage.getNormalForClosestNeighbors(x, y, validationPoints.normals[pointIdx]);
  }
}

//void ScanDatabaseElement::extractValidationPoints(int noOfPoints, MyPcl::NarfssKeypoint& keyPointDetector)
//{
  //resetValidationPoints();
  
  //VectorOfEigenVector3f tmpPoints;
  //keyPointDetector.setRangeImage(&rangeImage);
  //const pcl::PointCloud<pcl::InterestPoint>& keyPoints = keyPointDetector.getKeypoints();
  //for (size_t i=0; i<keyPoints.points.size(); ++i)
    //tmpPoints.push_back(keyPoints.points[i].getVector3fMap());
  //keyPointDetector.setRangeImage(NULL);
  
  //getUniformlyDistributedPoints(tmpPoints, noOfPoints, validationPoints);
  ////cout << PVARN(validationPoints.size());
//}


//void ScanDatabaseElement::extractValidationPoints(int noOfPoints, pcl::NarfKeypoint& keyPointDetector, float minSurfaceChange)
//{
  //resetValidationPoints();
  
  //VectorOfEigenVector3f tmpPoints;
  
  //pcl::RangeImageBorderExtractor borderExtractor = *keyPointDetector.getRangeImageBorderExtractor();
  //borderExtractor.setRangeImage(&rangeImage);
  //const float* surfaceChangeScores = borderExtractor.getSurfaceChangeScores();
  //for (size_t pointIdx=0; pointIdx<rangeImage.points.size(); ++pointIdx) {
    //if (surfaceChangeScores[pointIdx] > minSurfaceChange)
      //tmpPoints.push_back(rangeImage.getPoint(pointIdx).getVector3fMap());
  //}
  //borderExtractor.setRangeImage(NULL);

  ////keyPointDetector.setRangeImage(&rangeImage);
  ////const pcl::PointCloud<pcl::InterestPoint>& keyPoints = keyPointDetector.getInterestPoints();
  ////for (size_t i=0; i<keyPoints.points.size(); ++i)
    ////tmpPoints.push_back(keyPoints.points[i].getVector3fMap());
  ////keyPointDetector.setRangeImage(NULL);
  
  //getUniformlyDistributedPoints(tmpPoints, noOfPoints, validationPoints);
  ////cout << PVARN(validationPoints.size());
//}

float ScanDatabaseElement::getHistogramDistance(const std::vector<double>& histogram) const {
  if (dictionaryHistogram.size() != histogram.size()) {
    std::cerr << __PRETTY_FUNCTION__ << ": Histogram sizes do not match ("
              << dictionaryHistogram.size()<<" vs. "<<histogram.size()<<")\n";
    return -1.0f;
  }
  float distance=0.0f;
  for (size_t histogramIdx=1; histogramIdx<histogram.size(); ++histogramIdx) {
    //distance += fabsf(histogram1[histogramIdx]-histogram2[histogramIdx])*(1-weights[histogramIdx]);
    distance += fabsf(dictionaryHistogram[histogramIdx]-histogram[histogramIdx]);
  }
  return distance;
}


//// ==================================
//// ==========DATABASE START==========
//// ==================================

ScanDatabase::ScanDatabase() : ScanDatabase::BaseClass(), kdTreeForFeatures_(NULL)
{
  //kdTreeForFeatures_ = new pcl::KdTreeFLANN<pcl::Narf*>;
  databaseDirectory = ".";
}

ScanDatabase::~ScanDatabase()
{
  clear();
}

void ScanDatabase::clear()
{
  for (size_t idx=0; idx<size(); ++idx)
    delete at(idx);
  BaseClass::clear();
  name.clear();
  delete kdTreeForFeatures_;  kdTreeForFeatures_ = NULL;
  featureSources_.clear();
  databaseDirectory = ".";
}

void ScanDatabase::load(const std::string& directoryName)
{
  //std::cout << __PRETTY_FUNCTION__<<" called.\n";
  clear();
  databaseDirectory = directoryName;
  DIR* directory;
  if((directory=opendir(databaseDirectory.c_str())) == NULL) {
    std::cerr << "Could not open directory.\n";
    return;
  }
  std::vector<std::string> scan_file_names;
  struct dirent *dirEntry;
  while ((dirEntry = readdir(directory)) != NULL) {
    //std::cout << dirEntry->d_name;
    if (true || dirEntry->d_type==DT_REG)  // Only regular files
    {
      //std::cout << " is regular file\n";
      std::string fileName = dirEntry->d_name;
      
      // Ignore non-pcd files
      std::string extension = ".pcd";
      if (fileName.size()<=extension.size() || fileName.substr(fileName.size()-extension.size(), extension.size())!=extension)
        continue;
      
      // Ignore far ranges files
      extension = "_far_ranges.pcd";
      if (fileName.size()>=extension.size() && fileName.substr(fileName.size()-extension.size(), extension.size())==extension)
        continue;
      
      scan_file_names.push_back(fileName);
      //std::cout << PVARN(fileName);
    }
    else {
      //std::cout << " is not a regular file ("<<int(dirEntry->d_type)<<")\n";
    }
  }
  closedir(directory);
  std::sort(scan_file_names.begin(), scan_file_names.end());
  
  name = pcl::getFilenameWithoutPath(databaseDirectory);
  resize(scan_file_names.size());
  for (unsigned int scanIdx=0; scanIdx<scan_file_names.size(); ++scanIdx)
  {
    cout << "Importing "<<scan_file_names[scanIdx]<<".\n";
    at(scanIdx) = new ScanDatabaseElement;
    ScanDatabaseElement& scan = *at(scanIdx);
    scan.pointCloudFileName = databaseDirectory+"/"+scan_file_names[scanIdx];
    //scan.loadPointCloud();
    scan.loadRangeImage();
    scan.loadNARFs();
    scan.loadValidationPoints();
    scan.loadInfoFile();
    scan.loadDictionaryHistogram();
    
    //// Hack to write stuff for Michael
    //ScanDatabaseElement tmpScan = scan;
    //tmpScan.loadPointCloud();
    //pcl::PointCloud<pcl::PointXYZRGB> tmpCloud;
    //for (size_t pointIdx=0; pointIdx<tmpScan.pointCloud.points.size(); ++pointIdx) {
      //pcl::PointXYZRGB p;
      //p.getVector3fMap() = tmpScan.pointCloud.points[pointIdx].getVector3fMap();
      //p.rgb = 0.0f;
      //tmpCloud.points.push_back(p);
    //}
    //tmpCloud.width = tmpCloud.points.size();
    //tmpCloud.height = 1;
    //tmpCloud.sensor_origin_[0] = tmpScan.knownPoses["SLAM"](0,3);
    //tmpCloud.sensor_origin_[1] = tmpScan.knownPoses["SLAM"](1,3);
    //tmpCloud.sensor_origin_[2] = tmpScan.knownPoses["SLAM"](2,3);
    //tmpCloud.sensor_origin_[3] = tmpScan.knownPoses["SLAM"](3,3);
    //tmpCloud.sensor_orientation_ = tmpScan.knownPoses["SLAM"].rotation();
    //std::string tmpCloudFilename = "bla/"+scan_file_names[scanIdx];
    //pcl::io::savePCDFile(tmpCloudFilename.c_str(), tmpCloud);
  }
  configFileParams.clear();
  std::ifstream configFile((databaseDirectory + "/config.dat").c_str());
  if (!configFile)
    std::cerr << "Could not open config file.\n";
  else {
    //std::cout << "Now parsing config.dat\n";
    std::string paramName, paramValue;
    while (!configFile.eof()) {
      configFile >> paramName;
      std::getline(configFile, paramValue);
      //while (!paramValue.emtpy() && (paramValue[0]==' ' || paramValue[0]=='\"'))
      int startIdx = paramValue.find_first_not_of(" \""),
          endIdx = paramValue.find_last_not_of(" \"");
      if (startIdx>=0 && endIdx<int(paramValue.size()))
        paramValue = paramValue.substr(startIdx, endIdx-startIdx+1);
      else
        paramValue = "";
      
      //cout << PVARC(paramName)<<PVARN(paramValue);
      if (!paramName.empty())
        configFileParams[paramName] = paramValue;
    }
  }
 
  //cout << "Found "<<scan_file_names.size()<<" scans in directory \""<<databaseDirectory<<"\".\n";
}


void ScanDatabase::saveConfigFile()
{
  if (configFileParams.empty())
    return;
  std::ofstream file((databaseDirectory + "/config.dat").c_str());
  for (std::map<std::string, std::string>::const_iterator it=configFileParams.begin(); it!=configFileParams.end(); ++it)
    file << it->first << " " << it->second<<"\n";
  file.close();
}

void ScanDatabase::getMatches(const ScanDatabaseElement& scan, float maxDescriptorDistance, float minDistanceBetweenMatches,
                              std::vector<pcl::PointCorrespondences6DVector>& featureMatchesPerScan,
                              bool useKdtree, float maxDescriptorDistanceKdtree) const
{
  featureMatchesPerScan.clear();
  featureMatchesPerScan.resize(size());
  for (unsigned int featureIdx=0; featureIdx<scan.features.size(); ++featureIdx)
    getMatches(*scan.features[featureIdx], featureIdx, maxDescriptorDistance, minDistanceBetweenMatches,
               featureMatchesPerScan, useKdtree, maxDescriptorDistanceKdtree);
}


void ScanDatabase::getMatches(const pcl::Narf& feature, int featureId, float maxDescriptorDistance, float minDistanceBetweenMatches,
                              std::vector<pcl::PointCorrespondences6DVector>& featureMatchesPerScan,
                              bool useKdtree, float maxDescriptorDistanceKdtree) const
{
  if (!useKdtree)
  {
    for (unsigned int scanIdx=0; scanIdx<size(); ++scanIdx)
    {
      const ScanDatabaseElement& scan = *at(scanIdx);
      scan.getMatches(feature, featureId, maxDescriptorDistance, minDistanceBetweenMatches, featureMatchesPerScan[scanIdx]);
    }
  }
  else
  {
    //pcl::PointCorrespondence6D match;
    std::cerr << __PRETTY_FUNCTION__<<": useKdtree=true is not implemented yet!\n";
    (void)maxDescriptorDistanceKdtree;
    return;
  }
}

struct IndexWithDistance
{
  int index;
  float distance;
  inline bool operator<(const IndexWithDistance& other) const { return distance<other.distance; }
};

void ScanDatabase::orderScansRegardingHistogramSimilarity(const std::vector<double>& histogram, int maxNoOfScans, std::vector<int>& scanIndices) const {
  scanIndices.clear();
  std::vector<IndexWithDistance> indicesWithDistances;
  indicesWithDistances.resize(size());
  for (size_t scanIdx=0; scanIdx<size(); ++scanIdx) {
    indicesWithDistances[scanIdx].index = scanIdx;
    indicesWithDistances[scanIdx].distance = at(scanIdx)->getHistogramDistance(histogram);
  }
  std::sort(indicesWithDistances.begin(), indicesWithDistances.end());
  maxNoOfScans = std::min(maxNoOfScans, (int)indicesWithDistances.size());
  scanIndices.resize(maxNoOfScans);
  for (int i=0; i<maxNoOfScans; ++i)
    scanIndices[i] = indicesWithDistances[i].index;
}

void ScanDatabase::orderScansRegardingGlobalFeatureSimilarity(const ScanDatabaseElement& scan, int maxNoOfScans, std::vector<int>& scanIndices) const {
  scanIndices.clear();
  if (scan.narfsForCompleteScan.empty()) {
    std::cerr << __PRETTY_FUNCTION__<<": No global features available for given scan.\n";
    return;
  }
  std::vector<IndexWithDistance> indicesWithDistances;
  indicesWithDistances.resize(size());
  for (size_t scanIdx=0; scanIdx<size(); ++scanIdx) {
    const ScanDatabaseElement& scan2 = *at(scanIdx);
    indicesWithDistances[scanIdx].index = scanIdx;
    float& distance = indicesWithDistances[scanIdx].distance;
    distance = 1e10;
    for (int narfIdx1=0; narfIdx1<=0; ++narfIdx1) {
      for (int narfIdx2=0; narfIdx2<int(scan2.narfsForCompleteScan.size()); ++narfIdx2) {
        float descrDist = scan.narfsForCompleteScan[narfIdx1]->getDescriptorDistance(*scan2.narfsForCompleteScan[narfIdx2]);
        distance = std::min(distance, descrDist);
      }
    }
  }
  std::sort(indicesWithDistances.begin(), indicesWithDistances.end());
  maxNoOfScans = std::min(maxNoOfScans, (int)indicesWithDistances.size());
  scanIndices.resize(maxNoOfScans);
  for (int i=0; i<maxNoOfScans; ++i)
    scanIndices[i] = indicesWithDistances[i].index;
}

float* getSurfacePatch(const pcl::RangeImage& range_image, const Eigen::Isometry3f& pose, int pixel_size, float world_size,
                       bool interpolate, bool identify_far_ranges, int replace_unseen_by_nearest_neighbor_iterations)
{
  //MEASURE_FUNCTION_TIME;
  //if (debug) cout << __PRETTY_FUNCTION__<<" called.\n";
  
  float max_dist = 0.5f*world_size,
        cell_size = world_size/float (pixel_size);
  float world2cell_factor = 1.0f/cell_size,
        world2cell_offset = 0.5f*float (pixel_size)-0.5f;
  float cell2world_factor = cell_size,
        cell2world_offset = -max_dist + 0.5f*cell_size;
  Eigen::Isometry3f inverse_pose = pose.inverse (Eigen::Isometry);
  
  int no_of_pixels = pixel_size*pixel_size;
  float* surface_patch = new float[no_of_pixels];
  SET_ARRAY (surface_patch, -std::numeric_limits<float>::infinity (), no_of_pixels);
  
  Eigen::Vector3f sensor_pos = range_image.getSensorPos(),
                  sp2 = pose * sensor_pos;
  bool bounding_box_includes_sensor_pos = 
    sp2[0]>-max_dist && sp2[0]<max_dist && sp2[1]>-max_dist && sp2[1]<max_dist && sp2[2]>-max_dist && sp2[2]<max_dist;
  int min_x=0, max_x=int (range_image.width)-1,
      min_y=0, max_y=int (range_image.height)-1;
  if (!bounding_box_includes_sensor_pos) {
    float min_x_f=int (range_image.width)-1, max_x_f=0, min_y_f=int (range_image.height)-1, max_y_f=0;
    Eigen::Vector3f dest_point (max_dist, max_dist, max_dist);
    for (int x=0; x<=1; ++x)
    {
      dest_point[0] = -dest_point[0];
      for (int y=0; y<=1; ++y)
      {
        dest_point[1] = -dest_point[1];
        for (int z=0; z<=1; ++z)
        {
          dest_point[2] = -dest_point[2];
          Eigen::Vector3f src_point = inverse_pose * dest_point;
          float image_x, image_y;
          range_image.getImagePoint (src_point, image_x, image_y);
          min_x_f = (std::min) (min_x_f, image_x);
          max_x_f = (std::max) (max_x_f, image_x);
          min_y_f = (std::min) (min_y_f, image_y);
          max_y_f = (std::max) (max_y_f, image_y);
        }
      }
    }
    min_x = std::max (0, int (pcl_lrint (floor (min_x_f))-1)),
    max_x = std::min (int (range_image.width)-1, int (pcl_lrint (ceil (max_x_f))+1)),
    min_y = std::max (0, int (pcl_lrint (floor (min_y_f))-1)),
    max_y = std::min (int (range_image.height)-1, int (pcl_lrint (ceil (max_y_f))+1));
    //cout << "Searching through range image area of size "<<max_x-min_x<<"x"<<max_y-min_y<<".\n"
         //<< PVARN(range_image);
  }
  else {
    //cout << "The range image's sensor position is inside the given cube -> Must go through the complete range image.\n";
  }
  
  //Eigen::Vector3f position = inverse_pose.translation ();
  //int middle_x, middle_y;
  //range_image.getImagePoint (position, middle_x, middle_y);
  //int min_search_radius = 2;
  //bool still_in_range = true;
  //cout << "Starting radius search around "<<middle_x<<","<<middle_y<<"\n";
  //for (int radius=0;  still_in_range;  ++radius) 
  //{
    ////cout << PVARN (radius);
    //int x=middle_x-radius-1, y=middle_y-radius;  // Top left - 1
    //still_in_range = radius<min_search_radius;
    //for (int i=0; i<8*radius || (radius==0&&i==0); ++i)
    //{
      //if (i<=2*radius) ++x; else if (i<=4*radius) ++y; else if (i<=6*radius) --x; else --y;
      ////cout << x<<","<<y<<"  ";
  for (int y=min_y; y<=max_y && interpolate; ++y) {
    for (int x=min_x; x<=max_x; ++x) {
      Eigen::Vector3f point1, point2, point3;
      if (!range_image.isValid (x,y) || !range_image.isValid (x+1,y+1))
        continue;
      range_image.getPoint (x, y, point1);
      point1 = pose*point1;
      if (fabs (point1[2]) > max_dist)
        continue;
      
      range_image.getPoint (x+1, y+1, point2);
      point2 = pose*point2;
      if (fabs (point2[2]) > max_dist)
        continue;
      
      for (int triangle_idx=0; triangle_idx<=1; ++triangle_idx)
      {
        if (triangle_idx==0)  // First triangle
        {
          if (!range_image.isValid (x,y+1))
            continue;
          range_image.getPoint (x, y+1, point3);
        }
        else  // Second triangle
        {
          if (!range_image.isValid (x+1,y))
            continue;
          range_image.getPoint (x+1, y, point3);
        }
        point3 = pose*point3;
        if (fabs (point3[2]) > max_dist)
          continue;
        
        // Are all the points either left, right, on top or below the bottom of the surface patch?
        if ( (point1[0] < -max_dist  &&  point2[0] < -max_dist  &&  point3[0] < -max_dist) ||
             (point1[0] >  max_dist  &&  point2[0] >  max_dist  &&  point3[0] >  max_dist) ||
             (point1[1] < -max_dist  &&  point2[1] < -max_dist  &&  point3[1] < -max_dist) ||
             (point1[1] >  max_dist  &&  point2[1] >  max_dist  &&  point3[1] >  max_dist))
        {
          continue;
        }
        
        //still_in_range = true;
        
        // Now we have a valid triangle (point1, point2, point3) in our new patch
        float cell1_x = world2cell_factor*point1[0] + world2cell_offset,
              cell1_y = world2cell_factor*point1[1] + world2cell_offset,
              cell1_z = point1[2],
              cell2_x = world2cell_factor*point2[0] + world2cell_offset,
              cell2_y = world2cell_factor*point2[1] + world2cell_offset,
              cell2_z = point2[2],
              cell3_x = world2cell_factor*point3[0] + world2cell_offset,
              cell3_y = world2cell_factor*point3[1] + world2cell_offset,
              cell3_z = point3[2];
        
        int min_cell_x = (std::max) (0, int (pcl_lrint (ceil ( (std::min) (cell1_x, (std::min) (cell2_x, cell3_x)))))),
            max_cell_x = (std::min) (pixel_size-1, int (pcl_lrint (floor ( (std::max) (cell1_x,
                                                                       (std::max) (cell2_x, cell3_x)))))),
            min_cell_y = (std::max) (0, int (pcl_lrint (ceil ( (std::min) (cell1_y, (std::min) (cell2_y, cell3_y)))))),
            max_cell_y = (std::min) (pixel_size-1, int (pcl_lrint (floor ( (std::max) (cell1_y,
                                                                       (std::max) (cell2_y, cell3_y))))));
        if (max_cell_x<min_cell_x || max_cell_y<min_cell_y)
          continue;
        
        //cout << "Current triangle covers area of size "<<max_cell_x-min_cell_x<<"x"<<max_cell_y-min_cell_y<<".\n";
        
        // We will now do the following:
        //   For each cell in the rectangle defined by the four values above,
        //   we test if it is in the original triangle (in 2D).
        //   If this is the case, we calculate the actual point on the 3D triangle, thereby interpolating the result
        //   See http://www.blackpawn.com/texts/pointinpoly/default.html
        Eigen::Vector2f cell1 (cell1_x, cell1_y),
                        cell2 (cell2_x, cell2_y),
                        cell3 (cell3_x, cell3_y),
                        v0 = cell3 - cell1,
                        v1 = cell2 - cell1;
        float dot00 = v0.dot (v0),
              dot01 = v0.dot (v1),
              dot11 = v1.dot (v1),
              invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
        
        for (int cell_x=min_cell_x; cell_x<=max_cell_x; ++cell_x)
        {
          for (int cell_y=min_cell_y; cell_y<=max_cell_y; ++cell_y)
          {
            Eigen::Vector2f current_cell (cell_x, cell_y),
                            v2 = current_cell - cell1;
            float dot02 = v0.dot (v2),
                  dot12 = v1.dot (v2),
                  u = (dot11 * dot02 - dot01 * dot12) * invDenom,
                  v = (dot00 * dot12 - dot01 * dot02) * invDenom;
            bool point_in_triangle = (u > -0.01) && (v >= -0.01) && (u + v <= 1.01);
            
            //cout << "Is point ("<<current_cell[0]<<","<<current_cell[1]<<") in triangle "
                 //<< " ("<<cell1[0]<<","<<cell1[1]<<"), "
                 //<< " ("<<cell2[0]<<","<<cell2[1]<<"), "
                 //<< " ("<<cell3[0]<<","<<cell3[1]<<"): "
                 //<< (point_in_triangle ? "Yes":"No") << "\n";
            
            if (!point_in_triangle)
              continue;
            
            float new_value = cell1_z + u* (cell3_z-cell1_z) + v* (cell2_z-cell1_z);
            //if (fabs (new_value) > max_dist)
              //cerr << "WTF:"<<PVARN (new_value)<<"\n";
            
            float& value = surface_patch[cell_y*pixel_size + cell_x];
            if (pcl_isinf (value))
              value = new_value;
            else
              value = (std::min) (value, new_value);
          }
        }
      }
    }
  }
  
  for (int y=min_y; y<=max_y; ++y) {
    for (int x=min_x; x<=max_x; ++x) {
      if (!range_image.isValid(x, y))
        continue;
      Eigen::Vector3f point = range_image.getPoint(x, y).getVector3fMap();
      point = pose * point;
      if (fabs (point[2]) > max_dist)
        continue;
      int cell_x = lrintf(world2cell_factor*point[0] + world2cell_offset),
          cell_y = lrintf(world2cell_factor*point[1] + world2cell_offset);
      if (cell_x<0 || cell_x>=pixel_size || cell_y<0 || cell_y>=pixel_size)
        continue;
      float& value = surface_patch[cell_y*pixel_size + cell_x];
      
      if (pcl_isfinite (value) && interpolate)
        continue;
      
      value = (pcl_isfinite (value) ? std::min(value, point[2]) : point[2]);
    }
  }
  
  // Now find out, if there should be max ranges
  for (int cell_y=0; cell_y<pixel_size && identify_far_ranges; ++cell_y)
  {
    for (int cell_x=0; cell_x<pixel_size; ++cell_x)
    {
      int index= cell_y*pixel_size + cell_x;
      float& value = surface_patch[index];
      if (!pcl_isinf (value))
        continue;
      
      // Go through immediate neighbors
      bool is_background = false;
      for (int cell2_y=cell_y-1; cell2_y<=cell_y+1&&!is_background; ++cell2_y)
      {
        for (int cell2_x=cell_x-1; cell2_x<=cell_x+1; ++cell2_x)
        {
          if (cell2_x<0||cell2_x>=pixel_size||cell2_y<0||cell2_y>=pixel_size || (cell2_x==cell_x && cell2_y==cell_y))
            continue;
          float neighbor_value = surface_patch[cell2_y*pixel_size + cell2_x];
          if (pcl_isfinite (neighbor_value))
          {
            float cell_pos_x = cell_x + 0.6f* (cell_x-cell2_x),
                  cell_pos_y = cell_y + 0.6f* (cell_y-cell2_y);
            Eigen::Vector3f fake_point (cell2world_factor* (cell_pos_x)+cell2world_offset,
                                        cell2world_factor*cell_pos_y+cell2world_offset, neighbor_value);
            fake_point = inverse_pose*fake_point;
            float range_difference = range_image.getRangeDifference (fake_point);
            if (range_difference > max_dist)
            {
              //if (debug) cout << PVARN (range_difference);
              value = std::numeric_limits<float>::infinity ();
              is_background = true;
              break;
            }
          }
        }
      }
      if (is_background)
      {
        // Set all immediate -INFINITY neighbors to INFINITY
        for (int cell2_y=cell_y-1; cell2_y<=cell_y+1; ++cell2_y)
        {
          for (int cell2_x=cell_x-1; cell2_x<=cell_x+1; ++cell2_x)
          {
            if (cell2_x<0||cell2_x>=pixel_size||cell2_y<0||cell2_y>=pixel_size || (cell2_x==cell_x && cell2_y==cell_y))
              continue;
            int index2 = cell2_y*pixel_size + cell2_x;
            float& neighbor_value = surface_patch[index2];
            if (pcl_isinf (neighbor_value) && neighbor_value<0)
            {
              neighbor_value = std::numeric_limits<float>::infinity ();
            }
          }
        }
      }
    }
  }
  
  if (replace_unseen_by_nearest_neighbor_iterations != 0) {
    std::vector<int> invalid_indices;
    for (int cell_idx=0; cell_idx<no_of_pixels; ++cell_idx)
      if (!pcl_isfinite(surface_patch[cell_idx]))
        invalid_indices.push_back(cell_idx);
    
    std::vector<int> change_iterations;
    change_iterations.resize(no_of_pixels, 0);
    
    for (int iteration=1; !invalid_indices.empty(); ++iteration)
    {
      if (replace_unseen_by_nearest_neighbor_iterations>0 && iteration>replace_unseen_by_nearest_neighbor_iterations)
        break;
      
      int no_of_remaining_invalid_cells=0;
      for (int invalid_indices_idx=0; invalid_indices_idx<int(invalid_indices.size()); ++invalid_indices_idx) {
        int index = invalid_indices[invalid_indices_idx],
            y = index/pixel_size,
            x = index - y*pixel_size;
        float neighbor_values_sum = 0.0f;
        int no_of_neighbor_values = 0;
        for (int y2=std::max(0, y-1); y2<=std::min(y+1, pixel_size-1); ++y2)
        {
          for (int x2=std::max(0, x-1); x2<=std::min(x+1, pixel_size-1); ++x2)
          {
            if (x2<0 || x2>=pixel_size)
              continue;
            int index2 = y2*pixel_size + x2;
            if (!pcl_isfinite(surface_patch[index2]) || change_iterations[index2]==iteration)
              continue;
            neighbor_values_sum += surface_patch[index2];
            ++no_of_neighbor_values;
            //cout << x2<<","<<y2<<" is valid neighbor for "<<x<<","<<y<<".\n";
          }
        }
        if (no_of_neighbor_values>0) {
          surface_patch[index] = neighbor_values_sum/float(no_of_neighbor_values);
          change_iterations[index] = iteration;
          continue;
        }
        invalid_indices[no_of_remaining_invalid_cells] = index;
        ++no_of_remaining_invalid_cells;
      }
      invalid_indices.resize(no_of_remaining_invalid_cells);
      //cout << PVARN(no_of_remaining_invalid_cells);
      //cout << PVARN(iteration);
    }
  }
  
  return surface_patch;
}

bool extractDescriptor(pcl::Narf& narf, int descriptor_size)
{
  float weight_for_first_point = 2.0f; // The weight for the last point is always 1.0f
  int no_of_beam_points = narf.getNoOfBeamPoints();
  float weight_factor = -2.0f*(weight_for_first_point-1.0f) / ((weight_for_first_point+1.0f)*float(no_of_beam_points-1)),
        weight_offset = 2.0f*weight_for_first_point / (weight_for_first_point+1.0f);
  
  int surface_patch_pixel_size = narf.getSurfacePatchPixelSize();
  float surface_patch_world_size = narf.getSurfacePatchWorldSize();
  float surface_patch_rotation = narf.getSurfacePatchRotation();
  float* surface_patch = narf.getSurfacePatch();
  
  float* descriptor = narf.getDescriptor();
  if (descriptor_size != narf.getDescriptorSize())
  {
    narf.getDescriptorSize() = descriptor_size;
    delete descriptor;
    descriptor = new float[descriptor_size];
    narf.setDescriptor(descriptor);
  }
  float angle_step_size = deg2rad(360.0f)/narf.getDescriptorSize();
  //cout << PVARN(no_of_beam_points)<<PVARN(surface_patch_pixel_size);

  float cell_size = surface_patch_world_size/float(surface_patch_pixel_size),
        cell_factor = 1.0f/cell_size,
        cell_offset = 0.5f*(surface_patch_world_size - cell_size),
        max_dist = 0.5f*surface_patch_world_size,
        beam_point_factor = (max_dist-0.5f*cell_size) / no_of_beam_points;

  for (int descriptor_value_idx=0; descriptor_value_idx<narf.getDescriptorSize(); ++descriptor_value_idx)
  {
    float& descriptor_value = narf.getDescriptor()[descriptor_value_idx];
    descriptor_value = 0.0f;
    float angle = descriptor_value_idx*angle_step_size + surface_patch_rotation,
          beam_point_factor_x = sinf(angle)*beam_point_factor,
          beam_point_factor_y = -cosf(angle)*beam_point_factor;
     
    std::vector<float> beam_values (no_of_beam_points + 1);
    float current_cell = 0.0f;
    float weight_sum = 0.0f;
    for (int beam_point_idx=0; beam_point_idx<=no_of_beam_points; ++beam_point_idx)
    {
      float& beam_value = beam_values[beam_point_idx];

      float beam_point_x = beam_point_factor_x * beam_point_idx,
            beam_point_y = beam_point_factor_y * beam_point_idx;
      float beam_point_cell_x = cell_factor * (beam_point_x+cell_offset),
            beam_point_cell_y = cell_factor * (beam_point_y+cell_offset);
      //cout << PVARC(beam_point_idx)<<PVARC(beam_point_x)<<PVARC(beam_point_y)<<PVARC(beam_point_cell_x)<<PVARN(beam_point_cell_y);
      int cell_x=pcl_lrint(beam_point_cell_x), cell_y=pcl_lrint(beam_point_cell_y);
      beam_value = surface_patch[cell_y*surface_patch_pixel_size + cell_x];
      if (!pcl_isfinite(beam_value))
      {
        if (beam_value > 0.0f)
          beam_value = max_dist;
        else
          beam_value = -std::numeric_limits<float>::infinity ();
      }
    }
    for (int beam_value_idx=0; beam_value_idx<no_of_beam_points; ++beam_value_idx)
    {
      float beam_value1=beam_values[beam_value_idx];
      if (!pcl_isfinite(beam_value1))
        continue;
      
      int beam_value_idx2 = beam_value_idx;
      float beam_value2=0;
      do {
        ++beam_value_idx2;
        beam_value2 = beam_values[beam_value_idx2];
      }
      while (!pcl_isfinite(beam_value2) && beam_value_idx2<no_of_beam_points);
      
      if (!pcl_isfinite(beam_value2))
        continue;
      
      //std::cout << PVARC(descriptor_value_idx)<<PVARC(beam_value_idx)<<PVARC(beam_value1)<<PVARC(beam_value_idx2)<<PVARN(beam_value2);
      
      float current_weight = weight_factor*float(beam_value_idx) + weight_offset;
      float diff = beam_value2-beam_value1;
      //cout << "Point "<<beam_value_idx<<" with value "<<diff<<" has weight "<<current_weight<<" for angle "<<rad2deg(angle)<<".\n";
      current_cell += current_weight * diff;
      weight_sum += current_weight;
    }
    if (weight_sum > 0.0f)
      current_cell /= weight_sum;
    // Scaling for easy descriptor distances:
    current_cell = atan2(current_cell, max_dist)/deg2rad(180.0f);  // Scales the result to [-0.5, 0.5]
    //cout << "Direction: "<<sinf(angle)<<","<<-cosf(angle)<<". "<<PVARAC(angle)<<PVARN(current_cell);
    //descriptor_value = pow(fabsf(current_cell), 0.8f) / narf.getDescriptorSize();
    //if (current_cell < 0.0f)
      //descriptor_value = -descriptor_value;
    descriptor_value = current_cell;
  }
  return true;
}

//class FakeNarf : public pcl::Narf {
  //public:
    //using pcl::Narf::getBlurredSurfacePatch;
    //static float* getBlurredSurfacePatch (const pcl::Narf& narf, int new_pixel_size, int blur_radius) {
      //return static_cast<const FakeNarf*>(&narf)->getBlurredSurfacePatch(new_pixel_size, blur_radius);
    //}
//};

float* getBlurredSurfacePatch(const pcl::Narf& narf, int new_pixel_size, int blur_radius)
{
  int old_pixel_size = narf.getSurfacePatchPixelSize();
  float new_to_old_factor = float(old_pixel_size)/float(new_pixel_size);
  int new_size = new_pixel_size*new_pixel_size;
  
  float* new_surface_patch = new float[new_size];
  for (int y=0; y<new_pixel_size; ++y)
  {
    for (int x=0; x<new_pixel_size; ++x)
    {
      float& new_pixel = new_surface_patch[y*new_pixel_size + x];
      new_pixel = 0.0f;
      float weight = 0.0f;
      bool found_far_range = false;
      for (int y2=y-blur_radius; y2<=y+blur_radius; ++y2)
      {
        for (int x2=x-blur_radius; x2<=x+blur_radius; ++x2)
        {
          int old_x = pcl_lrint(floor(new_to_old_factor * float(x2))),
              old_y = pcl_lrint(floor(new_to_old_factor * float(y2)));
          if (old_x<0 || old_x>=old_pixel_size || old_y<0 || old_y>=old_pixel_size)
            continue;
          float old_pixel = narf.getSurfacePatch()[old_y*old_pixel_size + old_x];
          if (!pcl_isfinite(old_pixel))
          {
            if (old_pixel > 0)
              found_far_range = true;
            continue;
          }
          //float currentWeight = 1.0f;
          float currentWeight = (blur_radius>0 ? 1.0f - 0.75f*float(std::max(abs(x2-x),abs(y2-y)))/float(blur_radius) : 1.0f);
          new_pixel += currentWeight * old_pixel;
          weight += currentWeight;
        }
      }
      
      if (weight > 0.0f)
        new_pixel /= weight;
      else
      {
        if (found_far_range)
          new_pixel = std::numeric_limits<float>::infinity ();
        else
          new_pixel = -std::numeric_limits<float>::infinity ();
      }
    }
  }
  return new_surface_patch;
}

void ScanDatabaseElement::extractGlobalFeatures(float maximumRangeInDataset) {
  int no_measurement_replacement_radius = 0;
  int blur_radius = 1;
  float max_dist = 0.8*maximumRangeInDataset;
  int noOfSampledRotations = 50;
  const int patchSize = 50,
            descriptorSize = 36;
  
  pcl::VectorAverage3f vector_average;
  //float max_dist = 5.0f*averageRangeMeasurment;
  if (max_dist <= 0.0f) {
    max_dist = 30;
    cerr << "averageRangeMeasurment is not set => using maximum distance "<<max_dist<<".\n";
  }
  float max_dist_reciprocal = 1.0f/max_dist;
  Eigen::Vector3f sensorPos = rangeImage.getSensorPos();
  for (size_t point_idx=0; point_idx<rangeImage.points.size(); ++point_idx) {
    if (!rangeImage.isValid(point_idx))
      continue;
    Eigen::Vector3f point = rangeImage.points[point_idx].getVector3fMap();
    float distance = (sensorPos-point).norm();
    if (distance > max_dist)
      continue;
    float weight = distance*max_dist_reciprocal;
    //float weight = 1.0f;
    vector_average.add(point, weight);
  }
  Eigen::Vector3f normal;
  Eigen::Vector3f eigen_values, eigen_vector2, eigen_vector3;
  vector_average.doPCA(eigen_values, normal, eigen_vector2, eigen_vector3);
  if (normal.dot((vector_average.getMean()-sensorPos).normalized()) < 0.0f)
    normal *= -1.0f;
  Eigen::Vector3f point_on_plane = (normal.dot(vector_average.getMean()) - normal.dot(sensorPos))*normal + sensorPos;
  //cout << PVARC(point_on_plane)<<PVARN(normal);
  
  Eigen::Affine3f tmp;
  pcl::getTransformationFromTwoUnitVectorsAndOrigin ((fabsf(normal[0])<0.5f ? Eigen::Vector3f(1,0,0) : Eigen::Vector3f(0,1,0)),
                                                     normal, point_on_plane, tmp);
  Eigen::Isometry3f viewingPose;
  viewingPose.matrix() = tmp.matrix();
  
  float* patch = getSurfacePatch(rangeImage, viewingPose, patchSize, 2.0f*max_dist, true, false, no_measurement_replacement_radius);
  
  while (!narfsForCompleteScan.empty()) {
    delete narfsForCompleteScan.back();
    narfsForCompleteScan.pop_back();
  }
  narfsForCompleteScan.push_back(new pcl::Narf);
  pcl::Narf& narf = *narfsForCompleteScan.back();
  
  delete narf.getSurfacePatch();
  narf.setSurfacePatch(patch);
  narf.getSurfacePatchPixelSize() = patchSize;
  narf.getSurfacePatchWorldSize() = max_dist;
  narf.getSurfacePatchRotation() = 0.0f;
  narf.getTransformation() = viewingPose;
  narf.getPosition() = viewingPose.inverse (Eigen::Isometry).translation ();
  if (blur_radius > 0) {
    int new_pixel_size = narf.getSurfacePatchPixelSize();
    float* new_surface_patch = getBlurredSurfacePatch(narf, new_pixel_size, blur_radius);
    delete narf.getSurfacePatch();
    narf.setSurfacePatch(new_surface_patch);
    narf.getSurfacePatchPixelSize() = new_pixel_size;
  }
  extractDescriptor(narf, descriptorSize);
  
  bool use_rotation_sampling = true;
  std::vector<float> rotations;
  if (use_rotation_sampling)
  {
    for (int rotationIdx=1; rotationIdx<noOfSampledRotations; ++rotationIdx) {
      float rotation = rotationIdx * (2.0f*float(M_PI)/float(noOfSampledRotations));
      rotations.push_back(rotation);
    }
  }
  else
  {
    std::vector<float> strengths;
    narf.getRotations(rotations, strengths);
  }
  
  for (int rotationIdx=0; rotationIdx<int(rotations.size()); ++rotationIdx) {
    float rotation = rotations[rotationIdx];
    //cout << rad2deg(rotation)<<"deg\n";
    narfsForCompleteScan.push_back(new pcl::Narf);
    pcl::Narf& narf2 = *narfsForCompleteScan.back();
    narf2 = narf;
    narf2.getTransformation() = Eigen::AngleAxisf(-rotation, Eigen::Vector3f(0.0f, 0.0f, 1.0f))*narf2.getTransformation();
    narf2.getSurfacePatchRotation() = rotation;
    extractDescriptor(narf2, descriptorSize);
  }
}

#pragma GCC diagnostic ignored "-Wunused-parameter"  // Do not show warnings from ROS - for some weird reason
#pragma GCC diagnostic ignored "-Wstrict-aliasing"   // some stuff appears after the end of the code...
