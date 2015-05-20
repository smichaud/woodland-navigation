#include <iostream>
#include <fstream>
#include <dirent.h>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iomanip>
#pragma GCC diagnostic ignored "-Wunused-parameter"  // Do not show warnings from ROS
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#undef MEASURE_FUNCTION_TIME
#pragma GCC diagnostic warning "-Wunused-parameter"
using namespace std;

int main()
{
  //DIR* directory = opendir(".");
  //std::vector<std::string> scanFileNames;
  //struct dirent *dirEntry;
  //while ((dirEntry = readdir(directory)) != NULL) {
    //if (dirEntry->d_type == DT_REG)  // Only regular files
    //{
      //std::string file_name = dirEntry->d_name;
      //if (file_name.size() <= 4 || file_name.substr(file_name.size()-4, 4)!=".wrl")
        //continue;
      //scanFileNames.push_back(dirEntry->d_name);
    //}
  //}
  //closedir(directory);
  //std::sort(scanFileNames.begin(), scanFileNames.end());
  
  //for (unsigned int scanIdx=0; scanIdx<scanFileNames.size(); ++scanIdx)1 0 0 0 0 0 0 0 2 
  //{
    //stringstream outputFileNameSS;
    //outputFileNameSS << "scan_"<<std::setfill('0')<<std::setw(3)<<scanIdx+1<<".pcd";
    //string outputFileName = outputFileNameSS.str();
    //cout << "Output filename is \""<<outputFileName<<"\".\n";
    //ifstream file(scanFileNames[scanIdx].c_str());
    //string line;
    //float x, y, z;
    //pcl::PointCloud<pcl::PointWithViewpoint> cloud;
    //while (!file.eof()) {
      //getline(file, line);
      //stringstream ss(line);
      //ss >> z >> x >> y;  x*=-1; y*=-1;
      //if (ss.fail())
        //continue;
      ////cout << x<<", "<<y<<", "<<z<<"\n";
      //pcl::PointWithViewpoint point;
      //point.x=x;  point.y=y;  point.z=z;  point.vp_x=0;  point.vp_y=0;  point.vp_z=0;
      //cloud.points.push_back(point);
    //}
    //cloud.width = cloud.points.size();
    //cloud.height = 1;
    //cloud.is_dense = false;
    //pcl::io::savePCDFile(outputFileName, cloud);
  //}
  
  vector<float> numbers;
  std::ifstream posesFile("poses");
  while (true) {
    float number;
    posesFile >> number;
    if (posesFile.eof())
      break;
    if (posesFile.fail()) {
      posesFile.clear();
      int filePos = posesFile.tellg();
      posesFile.seekg(filePos+1);
      continue;
    }
    numbers.push_back(number);
    //cout << number << " " << std::flush;
  }
  for (unsigned int i=0, index=0; i<60; ++i) {
    ++index;
    float axisX=numbers[index++],
          axisY=numbers[index++],
          axisZ=numbers[index++],
          angle=numbers[index++];
    float x=numbers[index++],
          y=numbers[index++],
          z=numbers[index++];
    Eigen::Affine3f pose = Eigen::Translation3f(Eigen::Vector3f(x,y,z)) * Eigen::AngleAxisf(angle, Eigen::Vector3f(axisX, axisY, axisZ));

    Eigen::Affine3f coord;
    coord(0,0)= 0.0f; coord(0,1)= 0.0f; coord(0,2)=1.0f; coord(0,3)=0.0f;
    coord(1,0)=-1.0f; coord(1,1)= 0.0f; coord(1,2)=0.0f; coord(1,3)=0.0f;
    coord(2,0)= 0.0f; coord(2,1)=-1.0f; coord(2,2)=0.0f; coord(2,3)=0.0f;
    coord(3,0)= 0.0f; coord(3,1)= 0.0f; coord(3,2)=0.0f; coord(3,3)=1.0f;
    pose = pose * coord;
    
    float roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(pose, x, y, z, roll, pitch, yaw);
    stringstream outputFileNameSS;
    outputFileNameSS << "scan_"<<std::setfill('0')<<std::setw(3)<<i+1<<"_info.dat";
    string outputFileName = outputFileNameSS.str();
    cout << "Output filename is \""<<outputFileName<<"\".\n";
    ofstream file(outputFileName.c_str());
    file << "Odometry: "<<x<<" "<<y<<" "<<z<<" "<<roll<<" "<<pitch<<" "<<yaw<<"\n";
  }

}

#pragma GCC diagnostic ignored "-Wunused-parameter"  // Do not show warnings from ROS - for some weird reason
                                                     // some stuff appears after the end of the code...
