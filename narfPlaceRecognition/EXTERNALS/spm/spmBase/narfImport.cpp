#include "narfImport.h"
#include "narfWord.h"
#include "narfInstance.h"

#include "pcl/search/kdtree.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/make_shared.hpp>

NarfImport::NarfImport(std::vector<pcl::Narf* >* features, pcl::PointCloud<pcl::PointXYZ>* pcloud, SPModel* model)
{
    _features = features;
    _pcloud = pcloud;
    _model = model;
    _wordSize = 0.1;
    _usePointCloud = true;
}

NarfImport::NarfImport(std::vector<pcl::Narf* >* features, SPModel* model)
{
    _features = features;
    _model = model;
    _wordSize = 0.1;
    _usePointCloud = false;
}

NarfImport::~NarfImport()
{
}

void NarfImport::compute()
{
  pcl::search::KdTree<pcl::PointXYZ> kdtree(true);
  if(_usePointCloud && _pcloud->size() > 0) {
    std::cerr << PVAR( _pcloud->size()) << std::endl;
    boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > cloudPtr = _pcloud->makeShared ();
    std::cerr << PVAR( cloudPtr->size()) << std::endl;
    kdtree.setInputCloud (cloudPtr);
    std::cerr << "created kdTree" << std::endl;
  }

  for(size_t i = 0; i < _features->size(); ++i)
  {
    pcl::Narf*& narf = (*_features)[i];
    NarfInstance instance;
    instance.fid = 0;
    instance.pose = narf->getTransformation().inverse();

    //extracting word
    NarfWord* word = new NarfWord;
    instance.word = word;
    word->feature = *narf;


    if(_usePointCloud){
      //copy points in sphere around narf point
      Eigen::Vector3f pose = narf->getPosition();
  //     if(pose(2) < 0.1)
  //        continue;
      pcl::PointXYZ test_point (pose(0), pose(1), pose(2));
      double max_dist = _wordSize * 0.5;
  
      std::vector<int> k_indices;
      std::vector<float> k_distances;
      kdtree.radiusSearch (test_point, max_dist, k_indices, k_distances);
//       std::cerr << "proccessing narf feature " << i << " found " << k_indices.size() << " points." << std::endl;
  
      pcl::PointCloud<pcl::PointXYZ> neighbors;
      for(size_t j = 0; j < k_indices.size(); ++j)
      {
        neighbors.push_back(_pcloud->points[k_indices[j]]);
      }
      pcl::transformPointCloud(neighbors, word->pointCloud, instance.pose.inverse());
    }
    _model->addInstanceAndWord(instance);
  }
}

