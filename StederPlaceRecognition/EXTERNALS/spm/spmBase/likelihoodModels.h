#ifndef LIKELIHOOD_MODELS_H_
#define LIKELIHOOD_MODELS_H_

#include <iostream>
#include "pcl/kdtree/kdtree_flann.h"

// Forward declarations
namespace pcl {
  template <typename PointType>
  class PointCloud;

  class PointXYZ;
  class PointWithRange;
}

namespace LikelihoodModels
{
//! calculates a spherical gaussian likelihood with the paramter sigma of the model
double sphericalGaussian(pcl::PointCloud<pcl::PointXYZ>& model, pcl::PointCloud<pcl::PointXYZ>& data, double& sigma);
//! calculate likelihood based on range dependent sigma
double sphericalGaussian(pcl::PointCloud<pcl::PointXYZ>& model, pcl::PointCloud<pcl::PointXYZ>& data);

double sphericalGaussianFixSigma(pcl::PointCloud<pcl::PointXYZ>& model, pcl::PointCloud<pcl::PointXYZ>& data, double& sigma);

inline double sphericalGaussian(double& sigma, double& distance){
  return (1.0 / (sigma * sqrt(2*M_PI))) * exp(-0.5 * (distance / (sigma*sigma)));
};

inline double logGaussian(double& sigma, double& distance){
  return log(1.0 / (sigma * sqrt(2*M_PI))) + (-0.5 * (distance / (sigma*sigma)));
};

}  // end namespace

#endif  
