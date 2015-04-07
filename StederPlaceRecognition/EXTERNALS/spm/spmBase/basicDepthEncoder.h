#ifndef SPM_BASIC_WORD_H
#define SPM_BASIC_WORD_H

#include <vector>
#include <ostream>
#include <istream>
#include <Eigen/Core>

//#pragma GCC diagnostic ignored "-Wunused-parameter"  // Do not show warnings from PCL
#include "pcl/range_image/range_image_planar.h"
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
//#pragma GCC diagnostic warning "-Wunused-parameter"

#include "basicModel.h"

namespace Ais3dTools{
  
template<typename DescriptionType>
struct SpmDepthEncoderT{
  
  SpmDepthEncoderT(){

  }
  
  std::vector<Eigen::Vector3f> getClassColors(){
    std::vector<Eigen::Vector3f> colors;
    colors.push_back(Eigen::Vector3f(1.0, 0.0, 0.0));
    colors.push_back(Eigen::Vector3f(0.0, 1.0, 0.0));
    colors.push_back(Eigen::Vector3f(0.0, 0.0, 1.0));
    colors.push_back(Eigen::Vector3f(0.5, 0.5, 0.0));
    colors.push_back(Eigen::Vector3f(0.0, 0.5, 0.5));
    colors.push_back(Eigen::Vector3f(0.5, 0.5, 0.5));
    colors.push_back(Eigen::Vector3f(0.0, 0.0, 0.5));
    colors.push_back(Eigen::Vector3f(0.0, 0.5, 0.0));
    colors.push_back(Eigen::Vector3f(0.5, 0.0, 0.0));    
    colors.push_back(Eigen::Vector3f(0.8, 0.8, 0.8));
    return colors;
  }
  
  void simpleUniformWordCut(int cellSize, pcl::RangeImagePlanar& rangeImage, typename std::vector<BasicWordT<DescriptionType> >& words, typename std::vector<BasicInstanceT<DescriptionType> >& instances){
    for (int y=0; y < static_cast<int> (rangeImage.height); y+=cellSize)
    {
      for (int x=0; x < static_cast<int> (rangeImage.width); x+=cellSize)
      {
	BasicWordT<DescriptionType> word;
	pcl::RangeImagePlanar tmpCloud;
	int i = floor(cellSize / 2);
	Eigen::Vector3f centerPoint = rangeImage.points[(y+i)*rangeImage.width + (x+i)].getArray3fMap();
	for (int yi=0; yi < cellSize; yi++)
	  for (int xi=0; xi < cellSize; xi++)
	  {
	    int index = (y+yi)*rangeImage.width + (x+xi);
	    if(pcl::isFinite(rangeImage.points[index]) && fabs((centerPoint - rangeImage.points[index].getVector3fMap()).norm()) < 100.99)
	      tmpCloud.push_back(rangeImage.points[index]);
	  }
	  
	if( tmpCloud.points.size() < 9)
	  continue;
	// Placeholder for the 3x3 covariance matrix at each surface patch
	Eigen::Matrix3f covariance_matrix;
	Eigen::Matrix3f rotation_matrix;	
	Eigen::Vector3f eigenValues;
	// 16-bytes aligned placeholder for the XYZ centroid of a surface patch
	Eigen::Vector4f xyz_centroid;

	// Estimate the XYZ centroid
	pcl::compute3DCentroid (tmpCloud, xyz_centroid);

	// Compute the 3x3 covariance matrix
	pcl::computeCovarianceMatrix (tmpCloud, xyz_centroid, covariance_matrix);

	pcl::eigen33(covariance_matrix, rotation_matrix, eigenValues);	  
	
	//check direction of normal
	Eigen::Vector3f origin(rangeImage.sensor_origin_(0), rangeImage.sensor_origin_(1), rangeImage.sensor_origin_(2));
//         if((centerPoint - origin).dot(rotation_matrix.col(0)) > 0){
//           rotation_matrix.col(0) = -rotation_matrix.col(0);
//           rotation_matrix.col(1) = -rotation_matrix.col(1);	  
//         }
	
	Eigen::Affine3f pose = (Eigen::Affine3f) Eigen::Matrix3f::Identity(); //(Eigen::Affine3f)Eigen::Translation3f(xyz_centroid(0), xyz_centroid(1), xyz_centroid(2)) * (Eigen::Affine3f)rotation_matrix;
	pose = (Eigen::Affine3f)Eigen::Translation3f(centerPoint(0), centerPoint(1), centerPoint(2)) * (Eigen::Affine3f)rotation_matrix;

	pcl::transformPointCloud(tmpCloud, word.data, Eigen::Affine3f::Identity()); //pose.inverse()
	words.push_back(word);

	
	BasicInstanceT<DescriptionType> instance;
	instance.pose = pose;
	instance.word = &word;
	instance.fid  = word.id;
	instances.push_back(instance);
      }
    }
  }
  
  void simpleUniformWordCut(int cellSize, pcl::RangeImagePlanar& rangeImage, BasicModelT<DescriptionType>& model){
    for (int y=0; y < static_cast<int> (rangeImage.height); y+=cellSize)
    {
      for (int x=0; x < static_cast<int> (rangeImage.width); x+=cellSize)
      {
	pcl::RangeImagePlanar tmpCloud;
	BasicWordT<DescriptionType>* word = new BasicWordT<DescriptionType>;
	int i = floor(cellSize / 2);
	Eigen::Vector3f centerPoint = rangeImage.points[(y+i)*rangeImage.width + (x+i)].getArray3fMap();
	for (int yi=0; yi < cellSize; yi++)
	  for (int xi=0; xi < cellSize; xi++)
	  {
	    int index = (y+yi)*rangeImage.width + (x+xi);
	    if(pcl::isFinite(rangeImage.points[index]) && fabs((centerPoint - rangeImage.points[index].getVector3fMap()).norm()) < 100.99)
	      tmpCloud.push_back(rangeImage.points[index]);
	  }
	  
	if( tmpCloud.points.size() < 9)
	  continue;
	// Placeholder for the 3x3 covariance matrix at each surface patch
	Eigen::Matrix3f covariance_matrix;
	Eigen::Matrix3f rotation_matrix;	
	Eigen::Vector3f eigenValues;
	// Estimate the XYZ centroid
	Eigen::Vector4f xyz_centroid; 	// 16-bytes aligned placeholder for the XYZ centroid of a surface patch
	pcl::compute3DCentroid (tmpCloud, xyz_centroid);

	// Compute the 3x3 covariance matrix
	pcl::computeCovarianceMatrix (tmpCloud, xyz_centroid, covariance_matrix);
	pcl::eigen33(covariance_matrix, rotation_matrix, eigenValues);	  
	
	//check direction of normal
	Eigen::Vector3f origin(rangeImage.sensor_origin_(0), rangeImage.sensor_origin_(1), rangeImage.sensor_origin_(2));
//         if((centerPoint - origin).dot(rotation_matrix.col(0)) > 0){
//           rotation_matrix.col(0) = -rotation_matrix.col(0);
//           rotation_matrix.col(1) = -rotation_matrix.col(1);	  
//         }
	
	Eigen::Affine3f pose = (Eigen::Affine3f)Eigen::Translation3f(centerPoint(0), centerPoint(1), centerPoint(2)) * (Eigen::Affine3f)rotation_matrix;
	pcl::transformPointCloud(tmpCloud, word->data, pose.inverse()); //pose.inverse()
	
	BasicInstanceT<DescriptionType> instance;
	instance.pose = pose;
	instance.word = word;
	instance.fid  = word->id;
	model.addInstanceAndWord(instance);
      }
    }
  }
  
  
  pcl::PointCloud< pcl::PointWithRange > getUniformKeypoints(pcl::RangeImagePlanar& rangeImage, float resolution){
    
    std::map<int, std::map<int, std::map<int, pcl::PointCloud< pcl::PointWithRange > > > > supportPoints;
    pcl::PointCloud< pcl::PointWithRange > keyPoints;
    
    for (int y=0; y < static_cast<int> (rangeImage.height); ++y)
    {
      for (int x=0; x < static_cast<int> (rangeImage.width); ++x)
      {	
	Eigen::Vector3i index;	
	pcl::PointWithRange& point = rangeImage.points[(y)*rangeImage.width + (x)];
	index(0) = lrint(point.x / resolution);
	index(1) = lrint(point.y / resolution);
	index(2) = lrint(point.z / resolution);	
	supportPoints[index(0)][index(1)][index(2)].push_back(point);	
      }
    }
    
    for(std::map<int, std::map<int, std::map<int, pcl::PointCloud< pcl::PointWithRange > > > >::iterator it = supportPoints.begin(); it != supportPoints.end(); ++it)
      for(std::map<int, std::map<int, pcl::PointCloud< pcl::PointWithRange > > >::iterator itt = it->second.begin(); itt != it->second.end(); ++itt)      
        for(std::map<int, pcl::PointCloud< pcl::PointWithRange > >::iterator ittt = itt->second.begin(); ittt != itt->second.end(); ++ittt)
	{
	  Eigen::Vector4f xyz_centroid; 	// 16-bytes aligned placeholder for the XYZ centroid of a surface patch
	  pcl::PointCloud< pcl::PointWithRange >& cloud = ittt->second;
	  pcl::compute3DCentroid (cloud, xyz_centroid);
	  
	  pcl::PointWithRange point;
	  float mindistance = 2*resolution;
	  for(size_t i = 0; i < cloud.size(); ++i)
	  {
	    float distanceToCentroid = (cloud.points[i].x - xyz_centroid(0)) * (cloud.points[i].x - xyz_centroid(0)) + (cloud.points[i].y - xyz_centroid(1)) * (cloud.points[i].y - xyz_centroid(1));
	    if(distanceToCentroid < mindistance)
	    {
	      mindistance = distanceToCentroid;
	      point = cloud.points[i];
	    }
	  }
	  if(cloud.size() > 0 && point.range < 10.0)
	    keyPoints.push_back(point);	  
	}
    std::cerr << "found " << keyPoints.size() << " keyPoints for " << std::endl;
    return keyPoints;
  }
  
   void getMultiResolutionKeypoints(pcl::RangeImagePlanar& rangeImage,  std::vector<float>& resolutions, float maximumRange, std::vector<pcl::PointCloud< pcl::PointWithRange > >& keyPoints){
    
    std::vector<std::map<int, std::map<int, std::map<int, pcl::PointCloud< pcl::PointWithRange > > > > > supportPointsPerResolution;
    supportPointsPerResolution.resize(resolutions.size());
    keyPoints.resize(resolutions.size());
    
    for (int y=0; y < static_cast<int> (rangeImage.height); ++y)
    {
      for (int x=0; x < static_cast<int> (rangeImage.width); ++x)
      {	
	Eigen::Vector3i index;	
	pcl::PointWithRange& point = rangeImage.points[(y)*rangeImage.width + (x)];
	
	if(!pcl::isFinite(point))
	  continue;
	
	///decide resolution index based on range
	if(resolutions.size() == 0){
	  std::cerr << "No resolutions provided... " << std::endl;
	  return;
	}
	int rid = (int) ((point.range / maximumRange) * (float) resolutions.size());
	if(point.range > maximumRange || rid >= (int)resolutions.size())
	  rid = resolutions.size() - 1;	
// 	std::cerr << rid << " " << resolutions.size() << " " << resolutions[rid] << std::endl;
	float gridSize  = 1.4 * resolutions[rid];
	index(0) = lrint(point.x / gridSize);
	index(1) = lrint(point.y / gridSize);
	index(2) = lrint(point.z / gridSize);	
	supportPointsPerResolution[rid][index(0)][index(1)][index(2)].push_back(point);	
      }
    }
    int count = 0;
    for(size_t k = 0; k < supportPointsPerResolution.size(); ++k)
      for(std::map<int, std::map<int, std::map<int, pcl::PointCloud< pcl::PointWithRange > > > >::iterator it = supportPointsPerResolution[k].begin(); it != supportPointsPerResolution[k].end(); ++it)
	for(std::map<int, std::map<int, pcl::PointCloud< pcl::PointWithRange > > >::iterator itt = it->second.begin(); itt != it->second.end(); ++itt)      
	  for(std::map<int, pcl::PointCloud< pcl::PointWithRange > >::iterator ittt = itt->second.begin(); ittt != itt->second.end(); ++ittt)
	  {
	    Eigen::Vector4f xyz_centroid; 	// 16-bytes aligned placeholder for the XYZ centroid of a surface patch
	    pcl::PointCloud< pcl::PointWithRange >& cloud = ittt->second;
// 	    std::cerr << PVAR(cloud.size()) << " " << resolutions[k] << std::endl;
	    pcl::compute3DCentroid (cloud, xyz_centroid);
	    
	    pcl::PointWithRange point;
	    float mindistance = 2*resolutions[k];
	    for(size_t i = 0; i < cloud.size(); ++i)
	    {
	      float distanceToCentroid = (cloud.points[i].x - xyz_centroid(0)) * (cloud.points[i].x - xyz_centroid(0)) + (cloud.points[i].y - xyz_centroid(1)) * (cloud.points[i].y - xyz_centroid(1));
	      if(distanceToCentroid < mindistance)
	      {
		mindistance = distanceToCentroid;
		point = cloud.points[i];
	      }
	    }
	    if(cloud.size() > 0){
	      keyPoints[k].push_back(point);
	      count++;
	    }
	  }
    std::cerr << "found " << count << " keyPoints for " << std::endl;
  }
  
//   void cutCloudsForWords(pcl::RangeImagePlanar& rangeImage, pcl::PointCloud< pcl::PointWithRange >& keyPoints, std::vector<pcl::PointCloud< pcl::PointWithRange > >& result){
//     pcl::search::KdTree<pcl::PointWithRange> kdTree(true); ///true gives sorted results
//     kdTree.setInputCloud(rangeImage.makeShared()); ///TODO: avoid copying full range image with makeShared!!
//     for(size_t i = 0; i < keyPoints.size(); ++i)
//     {
//       std::vector< int > k_indices;
//       std::vector< float > k_sqr_distances;
//       kdTree.radiusSearch(keyPoints[i], resolution, k_indices, k_sqr_distances);
//      
//       if (k_indices.size() > 9){
// 	Eigen::Vector3f centerPoint = keyPoints[i].getArray3fMap();
// 	pcl::RangeImagePlanar tmpCloud;
// 	for(size_t j = 0; j < k_indices.size(); ++j)
// 	{
// 	  pcl::PointWithRange& p = rangeImage[k_indices[j]];
// 	  if(pcl::isFinite(p)) //&& (p.getVector3fMap() - centerPoint).norm() < wordRadius
// 	    tmpCloud.push_back(p);
// 	}
// 	Eigen::Affine3f pose;
// 	Eigen::Vector3f origin(rangeImage.sensor_origin_(0), rangeImage.sensor_origin_(1), rangeImage.sensor_origin_(2));
// 	BasicWordT<DescriptionType>* word = createWord(tmpCloud, origin, centerPoint, pose); 
//       }
//     }
//   }

  void createModelForKeyPoints(pcl::RangeImagePlanar& rangeImage, pcl::PointCloud< pcl::PointWithRange >& keyPoints, BasicModelT<DescriptionType>& model, float wordRadius, float wordResolution = 0.01, float minCoverage = 0.5){
    float resolution = wordRadius * 1.5;
    pcl::search::KdTree<pcl::PointWithRange> kdTree(true); ///true gives sorted results
    kdTree.setInputCloud(rangeImage.makeShared()); ///TODO: avoid copying full range image with makeShared!!
    for(size_t i = 0; i < keyPoints.size(); ++i)
    {
      std::vector< int > k_indices;
      std::vector< float > k_sqr_distances;
      kdTree.radiusSearch(keyPoints[i], resolution, k_indices, k_sqr_distances);
     
      if (k_indices.size() > 9){
	Eigen::Vector3f centerPoint = keyPoints[i].getArray3fMap();
	pcl::RangeImagePlanar tmpCloud;
	for(size_t j = 0; j < k_indices.size(); ++j)
	{
	  pcl::PointWithRange& p = rangeImage[k_indices[j]];
	  if(pcl::isFinite(p)) //&& (p.getVector3fMap() - centerPoint).norm() < wordRadius
	    tmpCloud.push_back(p);
	}
	Eigen::Affine3f pose;
	Eigen::Vector3f origin(rangeImage.sensor_origin_(0), rangeImage.sensor_origin_(1), rangeImage.sensor_origin_(2));
	BasicWordT<DescriptionType>* word = createWord(tmpCloud, origin, centerPoint, pose);
	
	///create description with fixed resolution
	float wordDiameter = 2*wordRadius;
	int width = wordDiameter / wordResolution;
	
	std::vector<float> descriptor;
	std::vector<float> counts;
	for(int j=0; j < (width*width); ++j){
	   descriptor.push_back(0.0f);
	   counts.push_back(0.0f);
	}

	for (int y=0; y < static_cast<int> (word->data.points.size()); ++y)
	{
	  pcl::PointWithRange& point = word->data.points[y];
	  if(pcl::isFinite(point))
	  {
	    int index_x = round((point.x) / wordResolution) + width/2;
	    int index_y = round((point.y) / wordResolution) + width/2;
	    if(index_x < 0 || index_y < 0 || index_x >= width || index_y >= width || fabs(point.z) > wordRadius){
// 	      std::cerr << "invalid index " << index_x << " or " << index_y << std::endl;

	    } else {
	      ///mean
	      descriptor[index_y * width + index_x] += point.z;
	      counts[index_y * width + index_x] += 1.0f;
	      ///min
	      //min(descriptor[index_y * width + index_x], point.z);
	      ///max
	      //descriptor[index_y * width + index_x] = max(descriptor[index_y * width + index_x], point.z);

	    }
	  }
	} 
	///normalize
	int validPointCount = 0;
	int minValidNeighbors = 7;
	for(int y=0; y < width; ++y)
	  for(int x=0; x < width; ++x){
	    int index = y * width + x;
	    if(counts[index] > 0.0f){
	      descriptor[index] /= counts[index];
	      validPointCount++;
	    } else {
	      descriptor[index] = wordRadius*2.1f;
	    }
	  }
	  
	if((float)validPointCount / (float) (width * width) < minCoverage) ///force a minimum coverage
	  continue;	
	
	///fill holes - FIRST iteration
	validPointCount = 0;
	for(int y=1; y < width-1; ++y)
	  for(int x=1; x < width-1; ++x){
	    int index = y * width + x;
	    if(descriptor[index] > wordRadius * 2.0f){
	      int validNeighbors = 0;
	      float mean = 0.0f;
	      for(int yi=y-1; yi <= y+1; ++yi)
		for(int xi=x-1; xi <= x+1; ++xi){
		  if(yi != y || xi != x){
		    if(descriptor[yi * width + xi] < wordRadius * 2.0f){
		      mean += descriptor[yi * width + xi];
		      validNeighbors++;
		    }
		  }
		}
	      if(validNeighbors >= minValidNeighbors){
		descriptor[index] = mean / (float)validNeighbors;
	      } 
	    }
	  }  
 
	word->data.points.clear();
	word->data.width = width;
	word->data.height = width;
	for(int i=0; i < (width); ++i)	
	  for(int j=0; j < (width); ++j){
	   int index = i * width + j;
	   pcl::PointWithRange p;
	   p.x = ((j - width /2) * wordResolution);
	   p.y = ((i - width /2) * wordResolution);
	   if(descriptor[index] > wordDiameter){
	     p.z = numeric_limits<float>::infinity();
	   } else 
	    p.z = descriptor[index];
	   p.range = p.z;
	   word->data.points.push_back(p);
	  }

	  BasicInstanceT<DescriptionType> instance;
	instance.pose = pose;
	instance.word = word;
	instance.fid  = word->id;
	model.addInstanceAndWord(instance);

      }
    }
  }
  
  void metricSizeUniformWordCut(pcl::RangeImagePlanar& rangeImage, BasicModelT<DescriptionType>& model, float wordRadius, float wordResolution = 0.01, float minCoverage = 0.5){
    float resolution = wordRadius * 1.5;
    pcl::PointCloud< pcl::PointWithRange > keyPoints = getUniformKeypoints(rangeImage, resolution);
   
    createModelForKeyPoints(rangeImage, keyPoints, model, wordRadius, wordResolution, minCoverage);
  }
 
  
  void metricSizeDynamicWordCut(pcl::RangeImagePlanar& rangeImage, BasicModelT<DescriptionType>& model, float minWordRadius, float minWordResolution = 0.01, float minCoverage = 0.5){
    float wordDiameter = 2*minWordRadius;
    int wordWidthInCells = wordDiameter / minWordResolution;
    int numOfLevels = 4;
    float kinectMaximumRange = 6.0;
    std::vector<float> wordRadius;
    std::vector<float> wordResolution;
    wordRadius.push_back(minWordRadius);
    wordResolution.push_back(minWordResolution);
    for(int i = 1; i < numOfLevels; ++i){
      wordRadius.push_back(wordRadius[i-1]*2);
      wordResolution.push_back(wordResolution[i-1]*2);
    }
/*
    for(int i = numOfLevels; i >= 0 ; --i){
      float resolution = wordRadius * 1.5;*/
    std::vector< pcl::PointCloud< pcl::PointWithRange > > keyPoints;
    getMultiResolutionKeypoints(rangeImage, wordRadius, kinectMaximumRange, keyPoints);
    
    for(int i = 0; i < numOfLevels; ++i)
      createModelForKeyPoints(rangeImage, keyPoints[i], model, wordRadius[i], wordResolution[i], minCoverage);
  }
  
  BasicWordT<DescriptionType>* createWord(pcl::RangeImagePlanar& input, Eigen::Vector3f& sensor_origin, Eigen::Vector3f& centerPoint, Eigen::Affine3f& pose)
  {
    BasicWordT<DescriptionType>* word = new BasicWordT<DescriptionType>;

    Eigen::Matrix3f covariance_matrix;
    Eigen::Matrix3f rotation_matrix;	
    Eigen::Vector3f eigenValues;
    
    /// Estimate the XYZ centroid
    Eigen::Vector4f xyz_centroid; 	/// 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    pcl::compute3DCentroid (input, xyz_centroid);

    /// Compute the 3x3 covariance matrix
    pcl::computeCovarianceMatrix (input, xyz_centroid, covariance_matrix);
    /// Compute SVD
    pcl::eigen33(covariance_matrix, rotation_matrix, eigenValues);	  
    
//     Eigen::Vector3f normal;
//     float eigenValue = 0.0f;
//     pcl::eigen33(covariance_matrix, eigenValue, normal);	 
//     pcl::PointXYZ p;
//     p.getVector3fMap() = centerPoint;
//     pcl::flipNormalTowardsViewpoint(p, sensor_origin(0), sensor_origin(1), sensor_origin(2), normal);
    
    ///check direction of normal
    Eigen::Vector3f tmpNormal = rotation_matrix.col(0);
    Eigen::Vector3f vp(sensor_origin(0) - centerPoint(0), sensor_origin(1) - centerPoint(1), sensor_origin(2) - centerPoint(2));    
    if(vp.dot(tmpNormal) < 0){
     tmpNormal *= -1;
//                 rotation_matrix.col(1) = -rotation_matrix.col(1);	  
    }   
    ///swap col(0) and col(2) to have the normal in z direction
    rotation_matrix.col(0) = rotation_matrix.col(2);
    rotation_matrix.col(2) = tmpNormal;
//     rotation_matrix.col(0) = tmpNormal;
    
    word->range = vp.norm();
    pose = (Eigen::Affine3f)Eigen::Translation3f(centerPoint(0), centerPoint(1), centerPoint(2)) * (Eigen::Affine3f)rotation_matrix;
    pcl::transformPointCloud(input, word->data, pose.inverse());   
    
    return word;
  }
  
};


} //end namespace

#endif