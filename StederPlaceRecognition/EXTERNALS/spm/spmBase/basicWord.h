#ifndef SPM_BASIC_WORD_T_H
#define SPM_BASIC_WORD_T_H

#include <vector>
#include <ostream>
#include <istream>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/features/narf.h"

namespace Ais3dTools{
  
  template<typename DescriptionType>
  struct BasicInstanceT;

  template<typename DescriptionType>
  class BasicWordT{
    public:
    typedef pcl::PointXYZ NWPoint;
    typedef pcl::PointCloud<NWPoint> NWPointCloud;
    
    BasicWordT(){

    }
    
    void writeBinary(std::ostream &file) const
    {   
      file.write((char*)&id, sizeof(id));      
      file.write((char*)&data.width, sizeof(data.width));
      file.write((char*)&data.height, sizeof(data.height));
      file.write((char*)&range, sizeof(range));
      for(unsigned int y = 0; y < data.height; y++)
	for(unsigned int x = 0; x < data.width; x++){
	  ///THIS BREAKS TEMPLATING!!!
	  int i = y*data.width + x;
	  file.write((char*)&data.points[i].x, sizeof(data.points[i].x));
	  file.write((char*)&data.points[i].y, sizeof(data.points[i].y));
	  file.write((char*)&data.points[i].z, sizeof(data.points[i].z));
	  file.write((char*)&data.points[i].range, sizeof(data.points[i].range));
	}  
    }

    void readBinary(std::istream &file)
    {
      file.read((char*)&id, sizeof(id));      
      file.read((char*)&data.width, sizeof(data.width));
      file.read((char*)&data.height, sizeof(data.height));
      file.read((char*)&range, sizeof(range));
      for(unsigned int y = 0; y < data.height; y++)
	for(unsigned int x = 0; x < data.width; x++){
	  ///THIS BREAKS TEMPLATING!!!
	  float x,y,z,range;
	  file.read((char*)&x, sizeof(x));
	  file.read((char*)&y, sizeof(y));
	  file.read((char*)&z, sizeof(z));
	  file.read((char*)&range, sizeof(range));
	  typename DescriptionType::PointType p;
	  p.x = x;
	  p.y = y;
	  p.z = z;
	  p.range = range;	
	  data.points.push_back(p);
	}
    };
    
    float getMeanRange()
    {
      float meanRange = 0.0f;
      float count = 0.0f;
      for(unsigned int y = 0; y < data.height; y++)
	for(unsigned int x = 0; x < data.width; x++){
	  if(pcl::isFinite(data.points[y*data.width + x])){
	    meanRange += data.points[y*data.width + x].range;
	    count     += 1.0f;
	  }
	}
      return meanRange / count;
    };
    
    
    // ---------------------------------
    // Member variables
    // ---------------------------------
    unsigned int              id;
    DescriptionType           data;
    NWPointCloud              pointCloud;
    float 	              range;
    typename std::vector<BasicInstanceT<DescriptionType> * > instances;
  
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
  };

}



#endif