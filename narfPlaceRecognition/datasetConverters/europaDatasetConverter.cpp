#include <iostream>
#include <fstream>
#include <dirent.h>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include "aislib/stuff/filesys_tools.h"
#include "mapping/map_graph/mapGraph.h"
#include <boost/filesystem.hpp> 

#pragma GCC diagnostic ignored "-Wunused-parameter"  // Do not show warnings from ROS
#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#undef MEASURE_FUNCTION_TIME
#pragma GCC diagnostic warning "-Wunused-parameter"
using namespace std;
using namespace AISNavigation;

bool writeBinary = true;

int main(int argc, char** argv)
{
  cout << "\n\n\n";
  if (optind >= argc) {
    cerr << "No directory given.\n";
    return 0;
  }
  std::string inputDirectoryName = argv[optind];
  std::string getBasename(const std::string& filename);
  
  DIR *directory;
  if((directory  = opendir(inputDirectoryName.c_str())) == NULL) {
    std::cerr << "Could not open directory.\n";
    return 0;
  }
  struct dirent *dirEntry;
  std::string mapFileName;
  while ((dirEntry = readdir(directory)) != NULL) {
    if (dirEntry->d_type != DT_REG)  // Only regular files
      continue;
    std::string fileName = dirEntry->d_name;
    if (getFileExtension(fileName) != "mapg")
      continue;
    mapFileName = fileName;
    break;
  }
  closedir(directory);
  
  if (mapFileName.empty()) {
    cerr << "Could not find *.mapg.\n";
    return 0;
  }
  
  MapGraph mapGraph = MapGraph(inputDirectoryName+"/"+mapFileName);
  if (mapGraph.vertices().empty()) {
    std::cerr << "Map graph is empty.\n";
    return 0;
  }
  
  string outputDirectoryName = inputDirectoryName+"/placeRecognitionDataset";
  boost::filesystem::create_directory(outputDirectoryName);
  cout << "Will use output directory \""<<outputDirectoryName<<"\".\n";
  
  for (MapGraph::VertexIDMap::iterator it=mapGraph.vertices().begin(); it!=mapGraph.vertices().end(); ++it) {
    int id = it->first;
    MapGraph::Vertex* vertex = static_cast<MapGraph::Vertex*>(it->second);
    if (vertex==NULL) {
      cout << "Node "<<id<<" does not exist.\n"; 
      continue;
    }
    LocalMap* localMap = static_cast<LocalMap*>(vertex->observation());
    if (localMap==NULL) {
      cout << "Node "<<id<<" does not have a local map.\n"; 
      continue;
    }
    Transformation3 pose = localMap->odomPose.get();
    string pointCloudFileName = localMap->getPcdName();
    //cout << "Node "<<id<<" has point cloud file \""<<pointCloudFileName<<"\".\n";
    
    //sensor_msgs::PointCloud2 pointCloudData;
    //Eigen::Vector4f sensorPos;  Eigen::Quaternionf sensorOrientation;
    //if (pcl::io::loadPCDFile(pointCloudFileName, pointCloudData, sensorPos, sensorOrientation) < 0) {
      //cerr << "Could not load point cloud for node "<<id<<".\n";
      //continue;
    //}
    pcl::PointCloud<pcl::PointXYZI> pointCloud;
    pcl::io::loadPCDFile(pointCloudFileName, pointCloud);
    
    stringstream fileNameStream;
    fileNameStream << outputDirectoryName << "/cloud_" << setfill ('0') << setw (6) << id;
    string fileNameBase = fileNameStream.str();
    string pcdFileName  = fileNameBase+".pcd";
    string infoFileName = fileNameBase+"_info.dat";
    string farRangesFileName = fileNameBase+"_far_ranges.pcd";
    
    cout << "Writing point cloud for node "<<id<<".\n";
    pointCloud.sensor_origin_.z() += 1.5;
    //sensorPos.z() += 1.5;
    
    
    // Create artificial far ranges
    Eigen::Vector4f sensorPos = pointCloud.sensor_origin_;
    Eigen::Quaternionf sensorOrientation = pointCloud.sensor_orientation_;
    pcl::PointCloud<pcl::PointWithViewpoint> farRanges;
    farRanges.sensor_origin_ = sensorPos;  farRanges.sensor_orientation_ = sensorOrientation;
    Eigen::Affine3f sensorPose =
      Eigen::Affine3f(Eigen::Translation3f(Eigen::Vector3f(sensorPos.x(), sensorPos.y(), sensorPos.z()))) *
      Eigen::Affine3f(sensorOrientation);
    pcl::RangeImage farRangesRangeImage;
    farRangesRangeImage.createEmpty(pcl::deg2rad(0.5), sensorPose, pcl::RangeImage::LASER_FRAME);
    for (int pointIdx=0; pointIdx<int(farRangesRangeImage.points.size()); ++pointIdx)
      farRangesRangeImage.points[pointIdx].range = 50.0f;
    farRangesRangeImage.recalculate3DPointPositions();
    for (int pointIdx=0; pointIdx<int(farRangesRangeImage.points.size()); ++pointIdx) {
      const pcl::PointWithRange& p1 = farRangesRangeImage.points[pointIdx];
      if (p1.z < 0.5f)
        continue;
      pcl::PointWithViewpoint p2;
      p2.x=p1.x, p2.y=p1.y, p2.z=p1.z,
           p2.vp_x=sensorPos.x(), p2.vp_y=sensorPos.y(), p2.vp_z=sensorPos.z();
      farRanges.points.push_back(p2);
    }
    farRanges.width  = farRanges.points.size();
    farRanges.height = 1;
    pcl::io::savePCDFile(farRangesFileName, farRanges, writeBinary);
    
    
    // Manipulate original point cloud a bit
    pcl::PointCloud<pcl::PointXYZI> tmpCloud;
    std::swap(pointCloud.points, tmpCloud.points);
    for (int pointIdx=0; pointIdx<int(tmpCloud.points.size()); ++pointIdx) {
      const pcl::PointXYZI& point = tmpCloud.points[pointIdx];
      int imageX, imageY;
      farRangesRangeImage.getImagePoint(point.getVector3fMap(), imageX, imageY);
      if (imageY > 0.6*farRangesRangeImage.height)
        continue;
      pointCloud.points.push_back(point);
    }
    pointCloud.width = pointCloud.points.size();
    pointCloud.height = 1;


    // Save point cloud to disc
    //if (pcl::io::savePCDFile(pcdFileName, pointCloudData, sensorPos, sensorOrientation, writeBinary) < 0) {
    if (pcl::io::savePCDFile(pcdFileName, pointCloud, writeBinary) < 0) {
      cerr << "Could not save point cloud as \""<<pcdFileName<<"\".\n";
      continue;
    }

    
    // Write robot pose to file
    Vector6 poseVec = pose.toVector();
    ofstream infoFile(infoFileName.c_str());
    infoFile << "Odometry: "<<poseVec[0]<<" "<<poseVec[1]<<" "<<poseVec[2]<<" "<<poseVec[3]<<" "<<poseVec[4]<<" "<<poseVec[5]<<"\n";
    infoFile << "IncrementalScanMatching: "<<poseVec[0]<<" "<<poseVec[1]<<" "<<poseVec[2]<<" "<<poseVec[3]<<" "<<poseVec[4]<<" "<<poseVec[5]<<"\n";
    infoFile << "SLAM: "<<poseVec[0]<<" "<<poseVec[1]<<" "<<poseVec[2]<<" "<<poseVec[3]<<" "<<poseVec[4]<<" "<<poseVec[5]<<"\n";
    infoFile.close();
  }
}

#pragma GCC diagnostic ignored "-Wunused-parameter"  // Do not show warnings from ROS - for some weird reason
                                                     // some stuff appears after the end of the code...
