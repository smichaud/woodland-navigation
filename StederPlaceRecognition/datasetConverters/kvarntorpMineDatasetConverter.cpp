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
  
  //ScanDatabase& scanDatabase = rangeImageMatching.scanDatabase;
  ScanDatabaseElement scanDatabaseElement;
  
  PointCloudType pointCloud;
  Vis::PointCloudSharedT<PointType> visPointCloud(&pointCloud);
  viewer.addObject(&visPointCloud);
  
  PointCloudf interestPoints;
  Vis::PointCloudSharedf visInterestPoints(&interestPoints);
  visInterestPoints.setColor(0.0, 1.0, 0.0); visInterestPoints.setPointSize(3.0);
  viewer.addObject(&visInterestPoints);
  
  bool needDraw=true;
  
  double x, y, tmp;
  
  Vector3f sensorpos(0.0, 0.0, 1.2);
  for (unsigned int dataSetIt=0; dataSetIt<4; ++dataSetIt) {
    int dataSetNo = 0, noOf3dScans=0, noOfFilesPer3dScan=0;
    if (dataSetIt==0) {
      dataSetNo=4;
      noOf3dScans = 57;
      noOfFilesPer3dScan = 226;
    }
    else if(dataSetIt==1) {
      dataSetNo=1;
      noOf3dScans = 74;
      noOfFilesPer3dScan = 226;
    }
    else break;
    
    
    for (int scanNo=1; scanNo<=noOf3dScans; ++scanNo) {
      cout << "Dataset "<<dataSetNo<<", scan "<<scanNo<<".\n";
      viewer.updateGL();
      application.processEvents();
      carmen_ipc_sleep(0.001);
      pointCloud.clear();
      
      ifstream odoFile(formatString("dat_mine%d/%03d/position.dat", dataSetNo, scanNo).c_str());
      //double odoTimestamp=0;
      double odoX=0, odoY=0, odoZ=0, odoRoll=0, odoPitch=0, odoYaw=0;
      //odoFile >> odoTimestamp >> odoX >> odoY >> odoZ >> odoRoll >> odoPitch >> odoYaw;
      SixDPose currentOdometryPose(0.001*odoX, -0.001*odoY, -0.001*odoZ, normAngle(odoRoll), -normAngle(odoPitch), -normAngle(odoYaw));
      scanDatabaseElement.odometryPose = currentOdometryPose;
      odoFile.close();
      
      for (int fileNo=1; fileNo<=noOfFilesPer3dScan; ++fileNo) {
        string fileName = formatString("dat_mine%d/%03d/scan%d.dat", dataSetNo, scanNo, fileNo);
        ifstream file(fileName.c_str());
        if (!file) {
          cout << "Unable to read file "<<fileName<<".\n";
          continue;
        }
        string tmpString;
        //getline(file, tmpString);
        //cout << tmpString<<"\n";
        double currentAngle;
        file >> tmpString >> tmpString >> tmpString >> tmpString >> tmpString >> tmpString >> tmpString >> tmpString >> currentAngle;
        //cout << PVARN(currentAngle);
        
        int line=1;
        while (true) {
          ++line;
          file >> x >> y >> tmp >> tmp;
          Vector3f point(0.001*y, 0.001*x, 0.0);
          point = Matrix3x3::rotY(DEG2RAD(currentAngle)) * point;
          point[2] += 0.4;
          
          if (file.eof()) break;
          if (file.fail()) {
            cout << "Failed in \""<<fileName<<"\" line "<<line<<".\n";
            exit(0);
          }
          pointCloud.add(point, sensorpos);
        }
      }
    }
  }


  //for (unsigned int scanNo=1; scanNo<=924; ++scanNo) {  //1-924
    
    //string fileName = formatString("scan3d_0_%d.3d", scanNo);
    //ifstream file(fileName.c_str());
    //if (!file) {
      //cout << "Unable to read file "<<fileName<<".\n";
      //continue;
    //}
    //line = 0;

    ////string line;
    ////while (getline(file, line)) {
      ////stringstream aux(line);
      ////aux >> x >> y >> z >> tmp >> tmp >> tmp;

    //while (true) {
      //++line;
      //file >> x >> y >> z >> tmp1 >> tmp2 >> tmp3;
      //Vector3f point(0.001*x + 0.32, -0.001*y, -0.001*z);
      
      //if (file.eof()) break;
      //if (file.fail()) {
        //cout << "Scan "<<scanNo<<", line "<<line<<" failed.\n";
        //exit(0);
      //}
      
      //float distance = point.norm();
      ////cout << distance<<", "<<std::flush;

      //if (distance > 29.5) {  // max range
        //pointCloud.maxRanges.push_back(PointWithSensorPosf(point, sensorpos));
      //}
      //else {
        //pointCloud.add(point, sensorpos);
      //}
    //}
    //file.close();
    
    //if (!scanDatabase.empty()) {
      ////cout << "Movement: "<<scanDatabase.back().odometryPose.inv()*scanDatabaseElement.odometryPose<<"\n";
      ////cout << PVARN(scanDatabaseElement.odometryPose);
    //}
    
//#if 1
    //scanDatabaseElement.pointCloud = new PointCloudWithSensorPosesf;
    //*scanDatabaseElement.pointCloud = pointCloud;
    //scanDatabaseElement.depthImage = new DepthImagef;
    //scanDatabaseElement.odometryPose = currentOdometryPose;
    //scanDatabaseElement.featureList = new VimripList;
    //scanDatabaseElement.validationPoints = new PointCloudf;
    
    //scanDatabase.push_back(scanDatabaseElement);
//#endif
    
    ////pointCloud.saveBinary(formatString("hanover2pointCloud_%d.pointCloud", scanNo).c_str());
  //}
  //scanDatabase.saveBinary("hanover2ScanDatabase.scanDatabase");

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
