#ifndef SPM_NARF_WORD_H
#define SPM_NARF_WORD_H

#include <vector>
#include <ostream>
#include <istream>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/features/narf.h"

// //PCL forward declarations
// namespace pcl {
//   struct PointXYZ;
//   struct Narf;
// 
//   template <typename PointT>
//   class PointCloud;
// }

struct NarfWord{
  typedef pcl::PointXYZ NWPoint;
  typedef pcl::PointCloud<NWPoint> NWPointCloud;
  
  NarfWord(){
    deleted = false;
    count = 0;
  }
  
  void writeBinary(std::ostream &file) const;
  void readBinary(std::istream &file);
  
  static struct NarfWordCmp{
    bool operator()(const NarfWord* item, const NarfWord* other) const{
        return item->count < other->count;
    }
  } NarfWordCmp_; 

  // ---------------------------------
  // Member variables
  // ---------------------------------
  unsigned int id;
  pcl::Narf feature;
  NWPointCloud pointCloud;
  bool deleted;
  unsigned int count;

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



#endif