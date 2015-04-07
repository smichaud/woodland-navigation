#include "likelihoodModels.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

double LikelihoodModels::sphericalGaussian(pcl::PointCloud<pcl::PointXYZ>& model, pcl::PointCloud<pcl::PointXYZ>& data, double& sigma)
{
  int maxNeighbors = 1;
  //double maxDistance = 100.0;
  //data associaton
  pcl::KdTreeFLANN<pcl::PointXYZ> dataKdtree;
  dataKdtree.setInputCloud (data.makeShared ());

  std::vector< std::vector<double> > coresspondingDistancesForDataPoints;
  coresspondingDistancesForDataPoints.resize(data.size());

  for(size_t i = 0; i < model.size(); i++)
    {
      std::vector<int> k_indices;
      std::vector<float> k_distances;

      k_indices.resize(maxNeighbors);
      k_distances.resize(maxNeighbors);
      dataKdtree.nearestKSearch(model, i, maxNeighbors, k_indices, k_distances);

      for(size_t j = 0; j < k_indices.size(); j++){
        int dataPointIndex = k_indices[j];
        coresspondingDistancesForDataPoints[dataPointIndex].push_back(k_distances[j]);
      }
    }


  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (model.makeShared ());
  
  for(size_t i = 0; i < data.size(); i++)
  {
     std::vector<double>& distances = coresspondingDistancesForDataPoints[i];
     if(distances.size() == 0){
      std::vector<int> k_indices;
      std::vector<float> k_distances;

      k_indices.resize(maxNeighbors);
      k_distances.resize(maxNeighbors);
      kdtree.nearestKSearch(data, i, maxNeighbors, k_indices, k_distances);

      for(size_t j = 0; j < k_distances.size(); j++)
        coresspondingDistancesForDataPoints[i].push_back(k_distances[j]);
      }
  }

  double likelihood = 10e-30;
  double uniformPrefix = log( 1.0 / (double) model.size());

  for(size_t i = 0; i < data.size(); i++)
  {
    double localLikelihood = 1e-30; //Numerically stable
    double prefix = (1.0 / (sigma * sqrt(2*M_PI)));
    std::vector<double>& distances = coresspondingDistancesForDataPoints[i];
    for(size_t j = 0; j <   distances.size(); j++){
       localLikelihood += (prefix) * exp(-0.5 * (distances[j] / (sigma*sigma)));
    }
    likelihood += uniformPrefix + log(localLikelihood);
  }
  return likelihood;
} 

double LikelihoodModels::sphericalGaussian(pcl::PointCloud<pcl::PointXYZ>& model, pcl::PointCloud<pcl::PointXYZ>& data)
{
  int maxNeighbors = 1;
  //double maxDistance = 100.0;
  //data associaton
  pcl::KdTreeFLANN<pcl::PointXYZ> dataKdtree;
  dataKdtree.setInputCloud (data.makeShared ());

  pcl::KdTreeFLANN<pcl::PointXYZ> modelKdtree;
  modelKdtree.setInputCloud (model.makeShared ());
  
  std::vector<int> k_indices;
  std::vector<float> k_distances;
  k_indices.resize(maxNeighbors);
  k_distances.resize(maxNeighbors);

  double likelihood = 10e-30;
  double uniformPrefix = log( 1.0 / (double) model.size());
  double sigmaPrefix = tan((0.25*M_PI/180));

  double likelihoodA = 0.0;  
  #pragma omp parallel for schedule(dynamic, 20) default(shared) reduction(+: likelihoodA) firstprivate(k_indices, k_distances) num_threads(12)
  for(size_t i = 0; i < model.size(); i++)
  {
    pcl::PointXYZ& p = model.points[i];
    double sigma = sigmaPrefix * p.getVector3fMap().norm() * 0.5;

    dataKdtree.nearestKSearch(p, maxNeighbors, k_indices, k_distances);
    double sqrDistance = k_distances[0];
    likelihoodA += uniformPrefix + logGaussian(sigma, sqrDistance);
  }

  double likelihoodB = 0.0;
  #pragma omp parallel for schedule(dynamic, 20) default(shared) reduction(+: likelihoodB) firstprivate(k_indices, k_distances) num_threads(12)
  for(size_t i = 0; i < data.size(); i++)
  {
    pcl::PointXYZ& p = data.points[i];                         
    double sigma = sigmaPrefix * p.getVector3fMap().norm() * 0.5;

    modelKdtree.nearestKSearch(p, maxNeighbors, k_indices, k_distances);
    double sqrDistance = k_distances[0];
    likelihoodB += uniformPrefix + logGaussian(sigma, sqrDistance);
  }
  likelihood = (likelihoodA + likelihoodB) / 2.0;

//   pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//   kdtree.setInputCloud (model.makeShared ());
//   
// //   #pragma omp parallel for schedule(dynamic, 20) default(shared) reduction(+: likelihood)
//   for(size_t i = 0; i < data.size(); i++)
//   {
//      pcl::PointXYZ& p = data.points[i];
//      kdtree.nearestKSearch(p, maxNeighbors, k_indices, k_distances);
//      for(size_t j = 0; j < k_indices.size(); j++){
//         int& dataPointIndex = k_indices[j];
//         double sigma = sigmaPrefix * p.getVector3fMap().norm() * 0.1;
//         double  localLikelihood = (1.0 / (sigma * sqrt(2*M_PI))) * exp(-0.5 * (k_distances[j] / (sigma*sigma)));
//         likelihood += uniformPrefix + log(localLikelihood);
//       }
//   }

/**
  for(size_t i = 0; i < data.size(); i++)
  {
    double sigma = sigmaPrefix * data.points[i].getVector3fMap().norm() * 0.1;
    double localLikelihood = 1e-30; //Numerically stable
    double prefix = (1.0 / (sigma * sqrt(2*M_PI)));
    std::vector<double>& distances = coresspondingDistancesForDataPoints[i];
    for(size_t j = 0; j <   distances.size(); j++){
       localLikelihood += (prefix) * exp(-0.5 * (distances[j] / (sigma*sigma)));
    }
    likelihood += uniformPrefix + log(localLikelihood);
  }
*/

  return likelihood;
} 


double LikelihoodModels::sphericalGaussianFixSigma(pcl::PointCloud<pcl::PointXYZ>& model, pcl::PointCloud<pcl::PointXYZ>& data, double& sigma)
{
  int maxNeighbors = 1;
  //double maxDistance = 100.0;
  //data associaton
  pcl::KdTreeFLANN<pcl::PointXYZ> dataKdtree;
  dataKdtree.setInputCloud (data.makeShared ());
  
  pcl::KdTreeFLANN<pcl::PointXYZ> modelKdtree;
  modelKdtree.setInputCloud (model.makeShared ());
  
  std::vector<int> k_indices;
  std::vector<float> k_distances;
  k_indices.resize(maxNeighbors);
  k_distances.resize(maxNeighbors);
  
  double likelihood = 10e-30;
  double uniformPrefix = log( 1.0 / (double) model.size());
  //double sigmaPrefix = tan((0.25*M_PI/180));
  
  double likelihoodA = 0.0;
  #pragma omp parallel for schedule(dynamic, 20) default(shared) reduction(+: likelihoodA) firstprivate(k_indices, k_distances) num_threads(12)
  for(size_t i = 0; i < model.size(); i++)
  {
    pcl::PointXYZ& p = model.points[i];    
    dataKdtree.nearestKSearch(p, maxNeighbors, k_indices, k_distances);
    double sqrDistance = k_distances[0];
    likelihoodA += uniformPrefix + logGaussian(sigma, sqrDistance);
  }
  
  double likelihoodB = 0.0;
  #pragma omp parallel for schedule(dynamic, 20) default(shared) reduction(+: likelihoodB) firstprivate(k_indices, k_distances) num_threads(12)
  for(size_t i = 0; i < data.size(); i++)
  {
    pcl::PointXYZ& p = data.points[i];   
    modelKdtree.nearestKSearch(p, maxNeighbors, k_indices, k_distances);
    double sqrDistance = k_distances[0];
    likelihoodB += uniformPrefix + logGaussian(sigma, sqrDistance);
  }
  likelihood = (likelihoodA + likelihoodB) / 2.0;
  
  return likelihood;
} 
