#ifndef HSM_MAP_INSTANCE_H
#define HSM_MAP_INSTANCE_H

#include "Eigen/Core"
#include "pcl/common/transforms.h"
#include "basics/transformationRepresentation.h"

struct NarfWord;

struct NarfInstance{
   unsigned int fid;
   Eigen::Affine3f pose;
   NarfWord* word;
   double observationRange;
   bool deleted;

   NarfInstance(){
      deleted = false;
   }

   void inline writeBinary(std::ostream &file) const {
      file.write((char*)&fid, sizeof(fid));
      float x, y, z; 
      float roll, pitch, yaw;
      x = pose.translation()(0);
      y = pose.translation()(1);
      z = pose.translation()(2);
      Eigen::Matrix3f m = pose.linear().matrix();
      Ais3dTools::TransformationRepresentation::getEulerFromMatrix(m, roll, pitch, yaw);
      file.write((char*)&x, sizeof(x));
      file.write((char*)&y, sizeof(x));
      file.write((char*)&z, sizeof(x));
      file.write((char*)&roll, sizeof(roll));
      file.write((char*)&pitch, sizeof(pitch));
      file.write((char*)&yaw, sizeof(yaw));            
      //pcl::saveBinary(pose.matrix(), file);
   }

   void inline readBinary(std::istream &file){
      file.read((char*)&fid, sizeof(fid));
      //pcl::loadBinary(pose.matrix(), file);
      float x, y, z;
      float roll, pitch, yaw;
      file.read((char*)&x, sizeof(x));
      file.read((char*)&y, sizeof(y));
      file.read((char*)&z, sizeof(z));
      file.read((char*)&roll, sizeof(roll));
      file.read((char*)&pitch, sizeof(pitch));
      file.read((char*)&yaw, sizeof(yaw));
      Eigen::Matrix3f m = Ais3dTools::TransformationRepresentation::getMatrixFromEuler(roll, pitch, yaw);
      pose.translation()(0) = x;
      pose.translation()(1) = y;
      pose.translation()(2) = z;
      pose.linear().matrix() = m; //Ais3dTools::TransformationRepresentation::getMatrixFromTranslationAndEuler(x, y, z, roll, pitch, yaw);      
   }
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif 
