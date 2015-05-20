#include <iostream>
using namespace std;

#include <qapplication.h>
#include <qaction.h>
#include <qprogressbar.h>
#include <qpushbutton.h>
#include <qlistbox.h>
#include <qlabel.h>

#include <carmen/carmen.h>
#include "util/filesysTools.h"
#include "tools/imageWidget/imageWidget.h"
#include "math/linalg/vector3.h"
#include "math/linalg/matrix3x3.h"
#include "math/linalg/matrix4x4.h"
#include "visualization/viewer/viewer.h"
#include "visualization/viewer/pointCloudShared.h"
#include "visualization/viewer/homer.h"
#include "ipcMsgManager/pointCloudMsgManager.h"
#include "ipcMsgManager/panTiltStatusMsgManager.h"
#include "math/geometry/pointCloud.h"
#include "math/geometry/pointCloudWithSensorPoses.h"
#include "rangeImageMatching.h"
#include "mainWidget.h"
#include "visualization/viewer/pointCloudList.h"
#include "objectRecognition/objectDataBase/objectDataBaseElement.h"
#include "util/stringTools.h"
#include "data_structs/poses_helper/sixDPose.h"
#include "data_structs/poses_helper/refPoses.h"
#include "data_structs/poses_helper/toroGraph.h"

typedef PointCloudWithSensorPosesf PointCloudType;
typedef PointCloudType::PointType PointType;

int noOfShownTransformations = 20;

int main(int argc, char** argv)
{
  carmen_ipc_initialize(argc, argv);
  
  RangeImageMatching rangeImageMatching;
  rangeImageMatching.getParams(argc, argv);
  
  QApplication application(argc,argv);
  Vis::Viewer viewer;
  application.setMainWidget(&viewer);
  viewer.setCaption("Cool file converter");
  viewer.show();
  
  ImageWidget depthImageWidget;
  depthImageWidget.useOriginalSize = true;
  depthImageWidget.setCaption("Depth image of cool file converter");
  
  ScanDatabase& scanDatabase = rangeImageMatching.scanDatabase;
  ScanDatabaseElement scanDatabaseElement;
  
  PointCloudType pointCloud;
  Vis::PointCloudSharedT<PointType> visPointCloud(&pointCloud);
  viewer.addObject(&visPointCloud);
  
  PointCloudf interestPoints;
  Vis::PointCloudSharedf visInterestPoints(&interestPoints);
  visInterestPoints.setColor(0.0, 1.0, 0.0); visInterestPoints.setPointSize(3.0);
  viewer.addObject(&visInterestPoints);
  
  bool needDraw=true;
  
  int x, y, z, tmp1, tmp2, tmp3;
  int line = 0;
  Vector3f sensorpos(0.0, 0.0, 1.2);
  ifstream odoFile("odometry_0_sync_interpol.dat");
  
  RefPoses refPoses;
  
  int usableScans = 0;
  for (unsigned int scanNo=1; scanNo<=924; ++scanNo) {  //1-924
    double odoTimestamp, odoX, odoY, odoZ, odoRoll, odoPitch, odoYaw;
    odoFile >> odoTimestamp >> odoX >> odoY >> odoZ >> odoRoll >> odoPitch >> odoYaw;
    SixDPose currentOdometryPose(0.001*odoX, -0.001*odoY, -0.001*odoZ, normAngle(odoRoll), -normAngle(odoPitch), -normAngle(odoYaw));
    scanDatabaseElement.odometryPose = currentOdometryPose;
    
    cout << PVARN(scanNo);
    viewer.updateGL();
    application.processEvents();
    carmen_ipc_sleep(0.001);
    pointCloud.clear();
    
    string fileName = formatString("scan3d_0_%d.3d", scanNo);
    ifstream file(fileName.c_str());
    if (!file) {
      cout << "Unable to read file "<<fileName<<".\n";
      continue;
    }
    line = 0;

    //string line;
    //while (getline(file, line)) {
      //stringstream aux(line);
      //aux >> x >> y >> z >> tmp >> tmp >> tmp;

    while (true) {
      ++line;
      file >> x >> y >> z >> tmp1 >> tmp2 >> tmp3;
      Vector3f point(0.001*x + 0.32, -0.001*y, -0.001*z);
      
      if (file.eof()) break;
      if (file.fail()) {
        cout << "Scan "<<scanNo<<", line "<<line<<" failed.\n";
        exit(0);
      }
      
      float distance = point.norm();
      //cout << distance<<", "<<std::flush;

      if (distance > 29.5) {  // max range
        pointCloud.maxRanges.push_back(PointWithSensorPosf(point, sensorpos));
      }
      else {
        pointCloud.add(point, sensorpos);
      }
    }
    file.close();
    
    if (!scanDatabase.empty()) {
      //cout << "Movement: "<<scanDatabase.back().odometryPose.inv()*scanDatabaseElement.odometryPose<<"\n";
      //cout << PVARN(scanDatabaseElement.odometryPose);
    }
    
#if 0
    scanDatabaseElement.pointCloud = new PointCloudWithSensorPosesf;
    *scanDatabaseElement.pointCloud = pointCloud;
    scanDatabaseElement.depthImage = new DepthImagef;
    //rangeImageMatching.extractRangeImage(pointCloud, *scanDatabaseElement.depthImage);
    //depthImageWidget.setRealImage(scanDatabaseElement.depthImage->data, scanDatabaseElement.depthImage->width, scanDatabaseElement.depthImage->height);
    scanDatabaseElement.odometryPose = currentOdometryPose;
    scanDatabaseElement.featureList = new VimripList;
    //rangeImageMatching.extractFeatures(*scanDatabaseElement.depthImage, *scanDatabaseElement.featureList);
    scanDatabaseElement.validationPoints = new PointCloudf;
    //interestPoints.clear();
    //rangeImageMatching.getInterestPoints(*scanDatabaseElement.depthImage, interestPoints);
    //rangeImageMatching.extractValidationPoints(*scanDatabaseElement.depthImage, *scanDatabaseElement.validationPoints, interestPoints);
    
    scanDatabase.push_back(scanDatabaseElement);
#endif
    
    //pointCloud.saveBinary(formatString("hanover2pointCloud_%d.pointCloud", scanNo).c_str());
    ++usableScans;
    refPoses.addPose(currentOdometryPose, odoTimestamp);
    pointCloud.writeVrml(formatString("pcloud%03d.wrl", usableScans-1).c_str());
  }
  refPoses.save("odom.ref");
  scanDatabase.saveBinary("hanover2ScanDatabase.scanDatabase");

  cout << "done\n";
  
  // Run main loop.
  while (viewer.isShown()) {
    application.processEvents();
    carmen_ipc_sleep(0.01);
    
    if (needDraw) {
      viewer.updateGL();
      needDraw = false;
    }
  }
}
