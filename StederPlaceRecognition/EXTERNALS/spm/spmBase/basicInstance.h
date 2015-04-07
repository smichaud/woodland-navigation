#ifndef SPM_BASIC_INSTANCE_H
#define SPM_BASIC_INSTANCE_H

#include "Eigen/Core"
#include "pcl/common/transforms.h"

namespace Ais3dTools{
  
  template<typename DescriptionType>
  class BasicWordT;

  template<typename DescriptionType>
  class BasicInstanceT{
    public:
    typedef BasicWordT<DescriptionType> BasicWord;

    unsigned int     fid;
    Eigen::Affine3f  pose;
    BasicWord *      word;

    BasicInstanceT(){
    }

    void inline writeBinary(std::ostream &file) const {
	file.write((char*)&word->id, sizeof(word->id));
	pcl::saveBinary(pose.matrix(), file);
    }

    void inline readBinary(std::istream &file){
	file.read((char*)&fid, sizeof(fid));
	pcl::loadBinary(pose.matrix(), file);
    }
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  
}

#endif 
