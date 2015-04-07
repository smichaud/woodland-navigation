#ifndef SPM_NARF_IMPORT_H
#define SPM_NARF_IMPORT_H

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/features/narf.h"
#include "spModel.h"

struct NarfImport{

  std::vector<pcl::Narf* >*          _features;
  pcl::PointCloud<pcl::PointXYZ>*    _pcloud;
  SPModel*                           _model;
  float                              _wordSize;

  NarfImport(std::vector<pcl::Narf* >* features, pcl::PointCloud<pcl::PointXYZ>* pcloud, SPModel* model);
  NarfImport(std::vector<pcl::Narf* >* features, SPModel* model);
  ~NarfImport();

  void compute();

  inline void setWordSize(float size)
  {
    _wordSize = size;
  };

  bool _usePointCloud;
};



#endif