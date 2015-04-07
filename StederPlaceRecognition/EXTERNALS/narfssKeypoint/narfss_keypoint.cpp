/* \author Bastian Steder - steder@informatik.uni-freiburg.de */

#include <iostream>
#include <vector>
#include "narfss_keypoint.h"
#include "farthestPoint.h"
#include <pcl/pcl_macros.h>
#include <pcl/common/polynomial_calculations.h>
#include <pcl/range_image/range_image.h>

inline double getTime() {
  struct timeval ts; 
  gettimeofday(&ts,0);
  return ts.tv_sec + ts.tv_usec*1e-6;
}

/////////////////////////////////////////////////////////////////////////
MyPcl::NarfssKeypoint::NarfssKeypoint (const pcl::RangeImage* range_image, float support_size) :
    BaseClass ()
{
  name_ = "NarfssKeypoint";
  setDescriptorSize();  // Set default value
  clearData ();
  setRangeImage (range_image);
  if (support_size > 0.0f)
    parameters_.support_size = support_size;
  debug_ = false;
}

/////////////////////////////////////////////////////////////////////////
MyPcl::NarfssKeypoint::~NarfssKeypoint ()
{
  //std::cerr << __PRETTY_FUNCTION__<<" called.\n";
  clearData ();
}

/////////////////////////////////////////////////////////////////////////
void
  MyPcl::NarfssKeypoint::clearData ()
{
  //std::cerr << __PRETTY_FUNCTION__<<" called.\n";
  
  for (size_t scale_space_idx = 1; scale_space_idx<range_image_scale_space_.size (); ++scale_space_idx)
    delete range_image_scale_space_[scale_space_idx];
  range_image_scale_space_.clear ();
  
  scale_space_usage_ranges_.clear ();
  
  for (size_t scale_space_idx = 0; scale_space_idx<descriptors_scale_space_.size (); ++scale_space_idx)
    delete descriptors_scale_space_[scale_space_idx];
  descriptors_scale_space_.clear();
  
  for (size_t scale_space_idx = 0; scale_space_idx<minimum_distance_values_scale_space_.size (); ++scale_space_idx)
    delete minimum_distance_values_scale_space_[scale_space_idx];
  minimum_distance_values_scale_space_.clear();
  
  for (size_t scale_space_idx = 0; scale_space_idx<neighbor_indices_scale_space_.size (); ++scale_space_idx)
    delete neighbor_indices_scale_space_[scale_space_idx];
  neighbor_indices_scale_space_.clear();
  
  for (size_t scale_space_idx = 0; scale_space_idx<calculated_at_this_scale_scale_space_.size (); ++scale_space_idx)
    delete calculated_at_this_scale_scale_space_[scale_space_idx];
  calculated_at_this_scale_scale_space_.clear();
  
  for (size_t scale_space_idx = 0; scale_space_idx<interest_image_scale_space_.size (); ++scale_space_idx)
    delete interest_image_scale_space_[scale_space_idx];
  interest_image_scale_space_.clear ();
  
  for (size_t scale_space_idx = 0; scale_space_idx<keypoint_indices_scale_space_.size (); ++scale_space_idx)
    delete keypoint_indices_scale_space_[scale_space_idx];
  keypoint_indices_scale_space_.clear ();
  
  original_scale_keypoint_indices_.clear();
  
  keypoints_.points.clear();
  keypoints_.width = keypoints_.height = 0;
}

/////////////////////////////////////////////////////////////////////////
void
MyPcl::NarfssKeypoint::setRangeImage (const pcl::RangeImage* range_image)
{
  clearData ();
  
  if (range_image == NULL)
    return;
  
  range_image_scale_space_.push_back(range_image);
}

/////////////////////////////////////////////////////////////////////////
void
MyPcl::NarfssKeypoint::calculateRangeImageScaleSpace ()
{
  //MEASURE_FUNCTION_TIME;
  
  //if (!parameters_.use_recursive_scale_reduction) {
    //scale_space_usage_ranges_.clear();
    //scale_space_usage_ranges_.push_back (0.0f);
    //return;
  //}
  
  if (range_image_scale_space_.empty() || range_image_scale_space_.size()>1)  // Nothing to compute or already done
    return;
  
  scale_space_usage_ranges_.clear();
  while (2.0f*range_image_scale_space_.back ()->getAngularResolution () < pcl::deg2rad (2.0f) &&
         range_image_scale_space_.size() < 127)
  {
    float scale_space_usage_range = parameters_.support_size / 
                                      tanf (static_cast<float> (parameters_.optimal_range_image_patch_size) *
                                            2.0f*range_image_scale_space_.back()->getAngularResolution ());
    if (scale_space_usage_range < parameters_.minimum_range)
      break;
    //std::cout << PVARN(scale_space_usage_range)<<PVARN(parameters_.minimum_range);
    scale_space_usage_ranges_.push_back(scale_space_usage_range);
    
    pcl::RangeImage* range_image_half = getRangeImage ().getNew ();
    range_image_scale_space_.back()->getHalfImage (*range_image_half);
    range_image_scale_space_.push_back(range_image_half);
    //std::cout << "Scale "<<range_image_scale_space_.size()-1<<" has angular resolution of "
              //<< pcl::rad2deg(range_image_half->getAngularResolution())<<"deg per pixel.\n";
  }
  scale_space_usage_ranges_.push_back(0.0f);
}

void 
MyPcl::NarfssKeypoint::calculateInterestImage ()
{
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";
  
  if (!interest_image_scale_space_.empty())  // Already done
    return;

  if (parameters_.support_size <= 0.0f)
  {
    std::cerr << __PRETTY_FUNCTION__<<": parameters_.support_size is not set!\n";
    return;
  }
  if (range_image_scale_space_.empty())
  {
    std::cerr << __PRETTY_FUNCTION__<<": no range image set.\n";
    return;
  }
  
  extractDescriptors ();
  
  while (interest_image_scale_space_.size()<range_image_scale_space_.size()) {
    interest_image_scale_space_.push_back(new std::vector<float>);
  }
  
  for (int scale_space_idx=int(range_image_scale_space_.size())-1; scale_space_idx>=0; --scale_space_idx) {
    const pcl::RangeImage& range_image = *range_image_scale_space_[scale_space_idx];
    const std::vector<char>& calculated_at_this_scale_image = *calculated_at_this_scale_scale_space_[scale_space_idx];
    std::vector<float>& interest_image = *interest_image_scale_space_[scale_space_idx];
    const std::vector<float>& descriptors = *descriptors_scale_space_[scale_space_idx];
    interest_image.resize(range_image.points.size());
    
    //double time_for_self_similarity_image = -pcl::getTime();
    #pragma omp parallel for num_threads(parameters_.max_no_of_threads) default(shared) schedule(guided, 10)
    for (int point_index=0; point_index<int(range_image.points.size()); ++point_index) {
      const pcl::PointWithRange& point = range_image.points[point_index];
      const float* descriptor = &descriptors[point_index*descriptor_size_];
      if (!pcl_isfinite (point.range) || !pcl_isfinite(descriptor[0])) {
        interest_image[point_index] = 0.0f;
        continue;
      }
      char calculated_at_this_scale = calculated_at_this_scale_image[point_index];
      if (calculated_at_this_scale != scale_space_idx)
      {
        int y = point_index/range_image.width,
            x = point_index - y*range_image.width;
        const pcl::RangeImage& half_range_image = *range_image_scale_space_[scale_space_idx+1];
        int half_x = std::min (x/2, int (half_range_image.width)-1),
            half_y = std::min (y/2, int (half_range_image.height)-1);
        int half_image_point_index = half_y*half_range_image.width + half_x;
        interest_image[point_index] = (*interest_image_scale_space_[scale_space_idx+1])[half_image_point_index];
        continue;
      }
      interest_image[point_index] = extractInterestValue(point_index, scale_space_idx);
    }
    //time_for_self_similarity_image += pcl::getTime();
    //std::cout << "Calculating interest values took "<<time_for_self_similarity_image
              //<< "s for a range image of size "<<range_image.width<<"x"<<range_image.height<<".\n";
    
    // The following reduces the score for descriptors with strong negative elements. This typically means an occlusion.
    // This is supposed to move the keypoints from the background to the foreground at object borders.
    //double time_for_additional_calculations = -pcl::getTime();
    if (parameters_.prevent_keypoints_in_background) {
      //if (debug_)
        //std::cout << "\n";
      const std::vector<float>& minimum_distance_values = *minimum_distance_values_scale_space_[scale_space_idx];
      #pragma omp parallel for num_threads(parameters_.max_no_of_threads) default(shared) schedule(guided, 100)
      for (size_t point_index=0; point_index<interest_image.size(); ++point_index) {
        float& interest_value = interest_image[point_index];
        float minimum_distance_value = minimum_distance_values[point_index];
        if (minimum_distance_value >= 0.0f)
          continue;
        //float correction_factor = 1.0f+minimum_distance_value;
        //float correction_factor = std::pow(1.0f+minimum_distance_value, 2);
        float correction_factor = std::sqrt(1.0f+minimum_distance_value);
        interest_value *= correction_factor;
        //if (debug_)
          //std::cout << PVARN(min_descriptor_value);
      }
    }
    filterBorderNoise(scale_space_idx);
    blurInterestImage(scale_space_idx);
    //time_for_additional_calculations += pcl::getTime();
    //std::cout << "Additional caclulations on the interest image took "<<time_for_additional_calculations
              //<< "s for a range image of size "<<range_image.width<<"x"<<range_image.height<<".\n";
  }
}

inline bool
isBetterKeypoint (const pcl::InterestPoint& p1, const pcl::InterestPoint& p2)
{
  return (p1.strength > p2.strength);
}

void 
MyPcl::NarfssKeypoint::calculateKeypoints ()
{
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";
  
  if (keypoints_.height > 0)  // Already done?
    return;
  
  //double time1 = getTime();
  calculateInterestImage ();
  //double time2 = getTime();
  
  if (interest_image_scale_space_.empty())
    return;
  
  while (keypoint_indices_scale_space_.size()<range_image_scale_space_.size()) {
    keypoint_indices_scale_space_.push_back(new std::vector<int>);
  }
  
  for (int scale_space_idx=int(range_image_scale_space_.size())-1; scale_space_idx>=0; --scale_space_idx) {
    const pcl::RangeImage& range_image = *range_image_scale_space_[scale_space_idx];
    const std::vector<float>& interest_image = *interest_image_scale_space_[scale_space_idx];
    const std::vector<char>& calculated_at_this_scale_image = *calculated_at_this_scale_scale_space_[scale_space_idx];
    std::vector<int>& keypoint_indices = *keypoint_indices_scale_space_[scale_space_idx];
    
    #pragma omp parallel for num_threads(parameters_.max_no_of_threads) default(shared) schedule(guided, 100)
    for (size_t point_index=0; point_index<interest_image.size (); ++point_index)
    {
      float interest_value = interest_image[point_index];
      if (interest_value<=0.0f || interest_value < parameters_.min_interest_value)
        continue;
      char calculated_at_this_scale = calculated_at_this_scale_image[point_index];
      if (calculated_at_this_scale != scale_space_idx)
        continue;
      
      if (scale_space_idx>0 && range_image.points[point_index].range >= scale_space_usage_ranges_[scale_space_idx-1])  // Resolution to bad?
        continue;
      
      if (parameters_.do_non_maximum_suppression) {
        int y = point_index/range_image.width,
            x = point_index - y*range_image.width;
        bool is_maximum = true;
        for (int y2=y-1; y2<=y+1 && is_maximum; ++y2) {
          for (int x2=x-1; x2<=x+1; ++x2) {
            if (!range_image.isInImage(x2, y2))
             continue;
            size_t point_index2 = y2*range_image.width + x2;
            const float interest_value2 = interest_image[point_index2];
            if (interest_value < interest_value2) {
              is_maximum = false;
              break;
            }
          }
        }
        if (!is_maximum)
          continue;
      }
      #pragma omp critical
      keypoint_indices.push_back(point_index);
    }
    //std::cout << "Found "<<keypoint_indices.size()<<" keypoints at scale "<<scale_space_idx<<".\n";
  }
  
  //double time3 = getTime();
  pcl::PointCloud<pcl::InterestPoint>::VectorType tmp_keypoints;
  #pragma omp parallel for num_threads(parameters_.max_no_of_threads) default(shared) schedule(guided, 100)
  for (int scale_space_idx=int(range_image_scale_space_.size())-1; scale_space_idx>=0; --scale_space_idx) {
    const pcl::RangeImage& range_image = *range_image_scale_space_[scale_space_idx];
    const std::vector<float>& interest_image = *interest_image_scale_space_[scale_space_idx];
    std::vector<int>& keypoint_indices = *keypoint_indices_scale_space_[scale_space_idx];
    for (size_t keypoint_indices_idx=0; keypoint_indices_idx<keypoint_indices.size (); ++keypoint_indices_idx)
    {
      int point_index = keypoint_indices[keypoint_indices_idx];
      float interest_value = interest_image[point_index];
      
      pcl::InterestPoint keypoint;
      keypoint.getVector3fMap() = range_image.points[point_index].getVector3fMap();
      keypoint.strength = interest_value;
      #pragma omp critical
      tmp_keypoints.push_back(keypoint);
    }
  }
  
  std::sort (tmp_keypoints.begin (), tmp_keypoints.end (), isBetterKeypoint);
  
  //double time4 = getTime();
  
  bool check_distance = parameters_.min_dist_between_keypoints_factor > 1e-5,
       restrict_no_of_keypoints = parameters_.max_no_of_keypoints > 0;
  
  if (!check_distance && !restrict_no_of_keypoints) {
    std::swap(keypoints_.points, tmp_keypoints);
  }
  else {
    size_t max_no_of_keypoints = (restrict_no_of_keypoints ? parameters_.max_no_of_keypoints : tmp_keypoints.size());
    float min_distance = parameters_.min_dist_between_keypoints_factor*parameters_.support_size;
    
    std::vector<float> points;
    for (size_t point_idx=0; point_idx<tmp_keypoints.size(); ++point_idx) {
      const pcl::InterestPoint& keypoint = tmp_keypoints[point_idx];
      points.push_back(keypoint.x);
      points.push_back(keypoint.y);
      points.push_back(keypoint.z);
    }
    std::vector<size_t> indices;
    FarthestPoint<float, 3>::subsample(points, max_no_of_keypoints, min_distance, indices);
    keypoints_.clear();
    for (size_t indices_idx=0; indices_idx<indices.size(); ++indices_idx)
      keypoints_.push_back(tmp_keypoints[indices[indices_idx]]);
    //std::cout << "Reduced number of keypoints from "<<tmp_keypoints.size()<<" to "<<keypoints_.size()<<".\n";
  }
  
  keypoints_.width = static_cast<uint32_t> (keypoints_.points.size ());
  keypoints_.height = 1;
  keypoints_.is_dense = true;
  
  //double time5 = getTime();

  //std::cout << "NARFSS: "<<PVARC(time2-time1) << PVARC(time3-time2) << PVARC(time4-time3) << PVARN(time5-time4);
}

const pcl::RangeImage& 
MyPcl::NarfssKeypoint::getRangeImage ()
{
  return (*range_image_scale_space_[0]);
}

void 
MyPcl::NarfssKeypoint::detectKeypoints (MyPcl::NarfssKeypoint::PointCloudOut& output)
{
  output.points.clear ();
  
  if (indices_)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": Sorry, usage of indices for the extraction is not supported for NARFs interest points (yet).\n\n";
    return;
  }
  
  if (range_image_scale_space_.empty())
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": RangeImage is not set. Sorry, the NARFs keypoint extraction works on range images, "
                   "not on normal point clouds.\n\n"
              << " Use setRangeImage (...).\n\n";
    return;
  }
  
  calculateKeypoints ();
  
  if (original_scale_keypoint_indices_.empty()) {
    const pcl::RangeImage& original_range_image = *range_image_scale_space_[0];
    for (size_t keypoint_idx=0; keypoint_idx<keypoints_.size(); ++keypoint_idx) {
      int x, y;
      original_range_image.getImagePoint(keypoints_[keypoint_idx].getVector3fMap(), x, y);
      original_scale_keypoint_indices_.push_back(y*original_range_image.width + x);
    }
  }
  
  //if (original_scale_keypoint_indices_.empty()) {
    //const pcl::RangeImage& original_range_image = *range_image_scale_space_[0];
    //for (int scale_space_idx=int(range_image_scale_space_.size())-1; scale_space_idx>0; --scale_space_idx) {
      //const pcl::RangeImage& range_image = *range_image_scale_space_[scale_space_idx];
      //std::vector<int>& keypoint_indices = *keypoint_indices_scale_space_[scale_space_idx];
      //int factor = std::pow(2, scale_space_idx);
      ////#pragma omp parallel for num_threads(parameters_.max_no_of_threads) default(shared) schedule(guided, 100)
      //for (size_t keypoint_indices_idx=0; keypoint_indices_idx<keypoint_indices.size (); ++keypoint_indices_idx)
      //{
        //int point_index = keypoint_indices[keypoint_indices_idx];
        //Eigen::Vector3f point = range_image.points[point_index].getVector3fMap();
        //int y = point_index/range_image.width,
            //x = point_index - y*range_image.width;
        //int original_start_x = factor*x,
            //original_end_x   = original_start_x + factor - 1,
            //original_start_y = factor*y,
            //original_end_y   = original_start_y + factor - 1;
        //int closest_idx = -1;
        //float closest_distance_squared = std::numeric_limits<float>::max();
        //for (int original_y=original_start_y; original_y<=original_end_y; ++original_y)
        //{
          //for (int original_x=original_start_x; original_x<=original_end_x; ++original_x)
          //{
            //const pcl::PointWithRange& point2 = original_range_image.getPoint(original_x, original_y);
            //if (!pcl_isfinite(point2.range))
              //continue;
            //float distance_squared = (point2.getVector3fMap()-point).squaredNorm();
            //if (distance_squared < closest_distance_squared)
            //{
              //closest_idx = original_y*original_range_image.width+original_x;
              //closest_distance_squared = distance_squared;
            //}
          //}
        //}
        //if (closest_idx < 0)
          //std::cerr << "\n\nWTF!?\n\n";
        //else
          //original_scale_keypoint_indices_.push_back(closest_idx);
        //int x2, y2;
        //original_range_image.getImagePoint(point, x2, y2);
        //std::cout << PVARC(closest_idx%original_range_image.width)<<PVARC(closest_idx/original_range_image.width)<<PVARC(x2)<<PVARN(y2);
      //}
    //}
    //original_scale_keypoint_indices_.insert(original_scale_keypoint_indices_.end(),
                                            //keypoint_indices_scale_space_[0]->begin(),
                                            //keypoint_indices_scale_space_[0]->end());
  //}
  output.points.clear();
  output.points.insert(output.points.end(), original_scale_keypoint_indices_.begin(), original_scale_keypoint_indices_.end());
  output.width = output.points.size();
  output.height = 1;
}

void 
MyPcl::NarfssKeypoint::compute (MyPcl::NarfssKeypoint::PointCloudOut& output)
{
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";
  detectKeypoints (output);
}

bool MyPcl::NarfssKeypoint::extractDescriptor(const pcl::RangeImage& range_image, int x, int y, float* descriptor,
                                              float& minimum_distance_value, int* neighbor_indices, int* descriptor_end_indices) const
{
  int blur_radius = parameters_.range_image_blur_radius;
  float optimal_neighbor_distance = parameters_.optimal_neighbor_distance_factor*parameters_.support_size,
        optimal_neighbor_distance_squared = powf(optimal_neighbor_distance, 2);
  
  int point_index = y*range_image.width + x;
  descriptor[0] = std::numeric_limits<float>::quiet_NaN ();
  minimum_distance_value = 1.0f;
  
  if (!range_image.isValid(x, y))
    return false;
  
  Eigen::Vector3f center_point = range_image.getPoint(x, y).getVector3fMap();
  float center_point_range = range_image.getPoint(x, y).range;
  
  Eigen::Isometry3f trans;
  trans.matrix() = range_image.getTransformationToViewerCoordinateFrame(center_point).matrix();
  
  const float pixel_step_size = 2.0f*float(blur_radius) + 1.0f;
  
  float radius = 0.5f*parameters_.support_size,
        radius_reciprocal = 1.0f/radius;
  
  float error_factor = (1.0f / (1.0f + std::pow(2.0f*parameters_.range_dependent_noise*center_point_range/parameters_.support_size, 2)));
  if (error_factor < 0.2)
    return false;
  
  float normalization_factor = error_factor * radius_reciprocal;
  
  //std::cout << PVARN(error_factor);
  
  bool descriptor_is_valid = true;
  for (int descriptor_value_idx=0; descriptor_value_idx<descriptor_size_&&descriptor_is_valid; ++descriptor_value_idx)
  {
    //if (debug_)
      //std::cout << "Descriptor value "<<descriptor_value_idx+1<<":\n";
    float& descriptor_value = descriptor[descriptor_value_idx];
    descriptor_value = 0.0f;
    int& neighbor_index = neighbor_indices[descriptor_value_idx];
    neighbor_index = -1;
    if (descriptor_end_indices != NULL)
      descriptor_end_indices[descriptor_value_idx] = -1;
    float sin_angle = lookupSinAngle_[descriptor_value_idx],
          cos_angle = lookupCosAngle_[descriptor_value_idx];
    
    float pixelDistanceToMiddle = 0.0f;
    float weight_sum = 0.0f;
    Eigen::Vector3f last_valid_beam_point(0.0f, 0.0f, 0.0f);
    float distance_to_center = 0.0f;
    bool optimal_neighbor_distance_reached = false;
    int last_point_index2 = point_index;
    int unobserved_counter = 0;
    
    float x2f=x, y2f=y;
    float step_size_x=sin_angle*pixel_step_size, step_size_y=-cos_angle*pixel_step_size;
    
    Eigen::Vector2f line_normal(cos_angle, sin_angle);
    float offset = line_normal.dot(Eigen::Vector2f(x,y));
    
    bool is_end_of_beam = false;
    while (!is_end_of_beam) {
      pixelDistanceToMiddle += pixel_step_size;
      x2f += step_size_x;
      y2f += step_size_y;
      int x2 = pcl_lrintf(x2f),
          y2 = pcl_lrintf(y2f);
      int point_index2 = y2*range_image.width + x2;
      
      if (last_point_index2==point_index2)
        continue;
      last_point_index2 = point_index2;
      
      bool found_far_range=false, at_least_one_neighbor_outside_image=false;
      Eigen::Vector3f blurred_point(0,0,0);
      int blurring_counter = 0;
      float closest_distance_above_optimal_neighbor_distance = std::numeric_limits<float>::max ();
      for (int y3=y2-blur_radius; y3<=y2+blur_radius; ++y3) {
        for (int x3=x2-blur_radius; x3<=x2+blur_radius; ++x3) {
          if (!range_image.isInImage(x3,y3)) {
            at_least_one_neighbor_outside_image = true;
            continue;
          }
          const pcl::PointWithRange& neighbor_point = range_image.getPoint(x3,y3);
          if (!pcl_isfinite(neighbor_point.range)) {
            if (neighbor_point.range > 0.0f)
              found_far_range = true;
            continue;
          }
          if (!optimal_neighbor_distance_reached) {
            float distance_2d = line_normal.dot(Eigen::Vector2f(x3,y3))-offset;  // Check if point lies on line
            if (std::abs(distance_2d)<=0.5f) {
              float distance_to_center_3d_squared = (center_point-neighbor_point.getVector3fMap()).squaredNorm();
              if (distance_to_center_3d_squared >= optimal_neighbor_distance_squared) {
                if (distance_to_center_3d_squared < closest_distance_above_optimal_neighbor_distance)
                {
                  closest_distance_above_optimal_neighbor_distance = distance_to_center_3d_squared;
                  neighbor_index = y3*range_image.width + x3;
                }
              }
            }
          }
          blurred_point += neighbor_point.getVector3fMap();
          ++blurring_counter;
        }
      }
      if (blurring_counter > 0)
        blurred_point /= blurring_counter;
      optimal_neighbor_distance_reached = neighbor_index>=0;
      
      bool unobserved_area = at_least_one_neighbor_outside_image && blurring_counter==0 && !found_far_range;
      
      float current_value = 0.0f;
      
      if (!unobserved_area) {
        if (found_far_range)
        {
          current_value = 2.0f;
          is_end_of_beam = true;
          //if (debug_)
            //std::cout << "Max range\n";
        }
        else if (blurring_counter==0)
        {
          {
            ++unobserved_counter;
            if (unobserved_counter>=3) {  // Restrict maximum number of unobserved points in a row
              unobserved_area = true;
            }
            else
              continue;
          }
        }
      }
      if (unobserved_area) {
        if (weight_sum > 0) {
          is_end_of_beam = true;
          current_value = (descriptor_value/distance_to_center)*(radius-distance_to_center);  // Assume continues surface | TODO: fix this
        }
        else {
          descriptor_is_valid = false;
          break;
        }
      }
      unobserved_counter = 0;
      float distance_to_last_valid_beam_point = 0.0f;
      if (!is_end_of_beam)
      {
        Eigen::Vector3f current_beam_point = trans * blurred_point;
        
        current_value = std::abs(current_beam_point[2] * normalization_factor);
        if (current_value > 1.0f)
          current_value = (2.0f - 1.0f/(3.0f*(current_value-1.0f)+1.0f));
        current_value *= 0.5f;  // Scale descriptor value to [-1.0,1.0]
        //current_value = sqrtf(current_value);
        current_value *= (current_beam_point[2]<0 ? -1.0f : 1.0f);
        
        distance_to_last_valid_beam_point = sqrtf(powf(current_beam_point[0]-last_valid_beam_point[0], 2) +
                                                  powf(current_beam_point[1]-last_valid_beam_point[1], 2));
        last_valid_beam_point = current_beam_point;
        is_end_of_beam = (distance_to_center+distance_to_last_valid_beam_point) >= radius;
      }
      
      float current_weight = (is_end_of_beam ? radius-distance_to_center : distance_to_last_valid_beam_point);
      
      minimum_distance_value = std::min(minimum_distance_value, (1.0f-(distance_to_center*radius_reciprocal))*current_value);
      //minimum_distance_value = std::min(minimum_distance_value, std::pow(1.0f-(distance_to_center*radius_reciprocal),2)*current_value);
      //minimum_distance_value = std::min(minimum_distance_value, (1.0f-std::pow(distance_to_center*radius_reciprocal,2))*current_value);
      
      distance_to_center += distance_to_last_valid_beam_point;
      weight_sum += current_weight;
      descriptor_value += current_weight*current_value;
      
      //if (debug_)
        //std::cout << PVARC(current_value)<<PVARC(current_weight)<<PVARC(minimum_distance_value)<<PVARN(descriptor_value/weight_sum);
      
      if (descriptor_end_indices != NULL)
        descriptor_end_indices[descriptor_value_idx] = point_index2;
    }
    if (!descriptor_is_valid) {
      descriptor[0] = std::numeric_limits<float>::quiet_NaN ();
      minimum_distance_value = 1.0f;
      return false;
    }
    if (weight_sum > 0.0f)
      descriptor_value /= weight_sum;
    descriptor_value *= 0.5f;  // Scale descriptor value to [-0.5,0.5]
  }
  
  //blurDescriptor(descriptor, descriptor_size_);

  return true;
}

void MyPcl::NarfssKeypoint::extractDescriptors()
{
  if (!descriptors_scale_space_.empty())  // Already done?
    return;
  
  calculateRangeImageScaleSpace();
  
  while (descriptors_scale_space_.size()<range_image_scale_space_.size()) {
    descriptors_scale_space_.push_back(new std::vector<float>);
    minimum_distance_values_scale_space_.push_back(new std::vector<float>);
    neighbor_indices_scale_space_.push_back(new std::vector<int>);
    calculated_at_this_scale_scale_space_.push_back(new std::vector<char>);
  }
  
  for (int scale_space_idx=int(range_image_scale_space_.size())-1; scale_space_idx>=0; --scale_space_idx) {
    const pcl::RangeImage& range_image = *range_image_scale_space_[scale_space_idx];
    //boost::shared_ptr<pcl::RangeImage> range_image_ptr(range_image_scale_space_[scale_space_idx]->getNew());
    //pcl::RangeImage& range_image = *range_image_ptr;
    //range_image_scale_space_[scale_space_idx]->getBlurredImage(2, range_image);
    
    std::vector<float>& descriptors = *descriptors_scale_space_[scale_space_idx];
    std::vector<float>& minimum_distance_values = *minimum_distance_values_scale_space_[scale_space_idx];
    std::vector<int>& neighbor_indices = *neighbor_indices_scale_space_[scale_space_idx];
    std::vector<char>& calculated_at_this_scale_image = *calculated_at_this_scale_scale_space_[scale_space_idx];
    float usage_range = scale_space_usage_ranges_[scale_space_idx];
     
    //std::cout << "Now extracting fast RF descriptors in every image point.\n";
    calculated_at_this_scale_image.resize(range_image.points.size ());
    descriptors.resize(descriptor_size_ * range_image.points.size ());
    minimum_distance_values.resize(range_image.points.size ());
    neighbor_indices.resize(descriptor_size_ * range_image.points.size ());
    
    //double time_for_descriptors = -pcl::getTime();
    #pragma omp parallel for num_threads(parameters_.max_no_of_threads) default(shared) schedule(guided, 10)
    for (int point_index=0; point_index<int(range_image.points.size()); ++point_index)
    {
      const pcl::PointWithRange& point = range_image.points[point_index];
      char& calculated_at_this_scale = calculated_at_this_scale_image[point_index];
      calculated_at_this_scale = -1;
      float* descriptor = &descriptors[point_index*descriptor_size_];
      float& minimum_distance_value = minimum_distance_values[point_index];
      minimum_distance_value = 1.0f;
      if (!pcl_isfinite (point.range) || point.range<parameters_.minimum_range ||
          (parameters_.maximum_range>0.0f&&point.range>parameters_.maximum_range)) {
        descriptor[0] = std::numeric_limits<float>::quiet_NaN ();
        continue;
      }
      int y = point_index/range_image.width,
          x = point_index - y*range_image.width;
      if (point.range < usage_range)  // Point is close enough that we can use the value calculated at a lower resolution
      {
        const pcl::RangeImage& half_range_image = *range_image_scale_space_[scale_space_idx+1];
        int half_x = std::min (x/2, int (half_range_image.width)-1),
            half_y = std::min (y/2, int (half_range_image.height)-1);
        int half_image_point_index = half_y*half_range_image.width + half_x;
        const float* descriptor2 = &((*descriptors_scale_space_[scale_space_idx+1])[half_image_point_index*descriptor_size_]);
        std::copy(descriptor2, descriptor2+descriptor_size_, descriptor);
        calculated_at_this_scale = (*calculated_at_this_scale_scale_space_[scale_space_idx+1])[half_image_point_index];
        continue;
      }
      calculated_at_this_scale = scale_space_idx;
      extractDescriptor(range_image, x, y, descriptor, minimum_distance_value, &neighbor_indices[point_index*descriptor_size_]);
    }
    //time_for_descriptors += pcl::getTime();
    //std::cout << "Extracting descriptors took "<<time_for_descriptors
              //<< "s for a range image of size "<<range_image.width<<"x"<<range_image.height<<".\n";
  }
}

float MyPcl::NarfssKeypoint::extractInterestValue(int point_index, int scale_space_idx) const
{
  float interest_value = 0.0f;
  
  const std::vector<float>& descriptors = *descriptors_scale_space_[scale_space_idx];
  const std::vector<int>& neighbor_indices = *neighbor_indices_scale_space_[scale_space_idx];
  
  const float* descriptor = &descriptors[point_index*descriptor_size_];
  if (!pcl_isfinite(descriptor[0]))
    return interest_value;
  interest_value = (parameters_.use_max_descr_dist_instead_of_min ? 0.0 : 1.0f);
  const int* current_neighbor_indices = &neighbor_indices[descriptor_size_*point_index];
  int last_neighbor_idx = -1;
  int no_of_valid_neighbors = 0;
  //if (debug_)
    //std::cout << "\n\nNeighbors descriptor distances:\n";
  for (int neighbor_id=0; neighbor_id<descriptor_size_; ++neighbor_id) {
    int point_index2 = current_neighbor_indices[neighbor_id];
    if (point_index2 < 0)
      continue;
    const float* descriptor2 = &descriptors[point_index2*descriptor_size_];
    if (!pcl_isfinite(descriptor2[0]))
      continue;
    ++no_of_valid_neighbors;
    if (point_index2==last_neighbor_idx)
      continue;
    
    float descriptor_distance = getDescriptorDistance(descriptor, descriptor2, descriptor_size_);
    
    //if (debug_)
      //std::cout << neighbor_id <<": "<<descriptor_distance<<" ";
    
    interest_value = (parameters_.use_max_descr_dist_instead_of_min ?
                        std::max(interest_value, descriptor_distance) :
                        std::min(interest_value, descriptor_distance));
  }
  //if (debug_)
    //std::cout << "\n\n";
  
  //const pcl::RangeImage& range_image = *range_image_scale_space_[scale_space_idx];
  //int y=point_index/range_image.width,
      //x=point_index-y*range_image.width;
  //float local_noise = 1.0f;
  //for (int y2=y-1; y2<=y+1; ++y2) {
    //for (int x2=x-1; x2<=x+1; ++x2) {
      //if (!range_image.isInImage(x2, y2) || (x2==x&&y2==y))
        //continue;
      //int point_index2 = y2*range_image.width+x2;
      //const float* descriptor2 = &descriptors[descriptor_size_*point_index2];
      //if (!pcl_isfinite(descriptor2[0]))
        //continue;
      //float descriptor_distance = getDescriptorDistance(descriptor, descriptor2, descriptor_size_);
      //if (descriptor_distance < local_noise)
        //local_noise=descriptor_distance;
    //}
  //}
  ////interest_value = local_noise;
  //interest_value *= std::pow(1.0f-local_noise, 5);
  ////if (local_noise > 0.05)
    ////interest_value = 0;
  
  float fractionValidNeighbors = float(no_of_valid_neighbors)/float(descriptor_size_);
  if (fractionValidNeighbors < 0.5f)
    interest_value = 0.0f;
  
  if (debug_)
    std::cout << PVARN(interest_value);

  return interest_value;
}

void MyPcl::NarfssKeypoint::filterBorderNoise(int scale_space_idx) {
  //return;
  //MEASURE_FUNCTION_TIME;
  
  const pcl::RangeImage& range_image = *range_image_scale_space_[scale_space_idx];
  const std::vector<char>& calculated_at_this_scale_image = *calculated_at_this_scale_scale_space_[scale_space_idx];
  std::vector<float>& interest_image = *interest_image_scale_space_[scale_space_idx];
  std::vector<float> tmp_interest_image;
  tmp_interest_image.resize(range_image.width*range_image.height);
  
  // TODO: Not really satisfactory yet:
  #pragma omp parallel for num_threads(parameters_.max_no_of_threads) default(shared) schedule(guided, 10)
  for (unsigned int point_index=0; point_index<range_image.points.size(); ++point_index)
  {
    float& value = tmp_interest_image[point_index];
    value = interest_image[point_index];
    char calculated_at_this_scale = calculated_at_this_scale_image[point_index];
    if (value<1e-5 || calculated_at_this_scale!=scale_space_idx)
      continue;
    int y = point_index/range_image.width,
        x = point_index - y*range_image.width;
    for (int y2=y-1; y2<=y+1; ++y2) {
      for (int x2=x-1; x2<=x+1; ++x2) {
        int point_index2 = y2*range_image.width + x2;
        float neighbor_value = interest_image[point_index2];
        if (neighbor_value < 1e-5)
          continue;
        value = std::min(value, interest_image[point_index2]);
      }
    }
  }
  std::swap(tmp_interest_image, interest_image);
}

void MyPcl::NarfssKeypoint::blurInterestImage(int scale_space_idx) {
  //return;
  //MEASURE_FUNCTION_TIME;

  const pcl::RangeImage& range_image = *range_image_scale_space_[scale_space_idx];
  const std::vector<char>& calculated_at_this_scale_image = *calculated_at_this_scale_scale_space_[scale_space_idx];
  std::vector<float>& interest_image = *interest_image_scale_space_[scale_space_idx];
  std::vector<float> tmp_interest_image;
  tmp_interest_image.resize(range_image.width*range_image.height);
  
  int no_of_iterations = parameters_.interest_image_blur_radius;
  if (no_of_iterations <= 0)
    return;
  float radius_squared_reciprocal = 1.0f / std::pow(0.5f*parameters_.support_size, 2);
  
  for (int iteration=0; iteration<no_of_iterations; ++iteration) {
    #pragma omp parallel for num_threads(parameters_.max_no_of_threads) default(shared) schedule(guided, 10)
    for (unsigned int point_index=0; point_index<range_image.points.size(); ++point_index)
    {
      float& value = tmp_interest_image[point_index];
      value = interest_image[point_index];
      char calculated_at_this_scale = calculated_at_this_scale_image[point_index];
      //if (calculated_at_this_scale < 0)
        //continue;
      if (value<1e-5 || calculated_at_this_scale!=scale_space_idx)
        continue;
      const Eigen::Vector3f point = range_image.points[point_index].getVector3fMap();
      int y = point_index/range_image.width,
          x = point_index - y*range_image.width;
      float weightSum = 0.0f;
      value = 0.0f;
      for (int y2=y-1; y2<=y+1; ++y2) {
        for (int x2=x-1; x2<=x+1; ++x2) {
          if (!range_image.isValid(x2, y2))
            continue;
          int point_index2 = y2*range_image.width + x2;
          float distance_squared = (point-range_image.points[point_index2].getVector3fMap()).squaredNorm();
          //float weight = 1.0f;
          float weight = 1.0f/(1.0f + distance_squared*radius_squared_reciprocal);
          value += weight*interest_image[point_index2];
          weightSum += weight;
        }
      }
      value /= float(weightSum);
    }
    //std::copy(tmp_interest_image.begin(), tmp_interest_image.end(), interest_image.begin());
    std::swap(tmp_interest_image, interest_image);
  }
}

void MyPcl::NarfssKeypoint::updateLookupTables()
{
  lookupSinAngle_.clear();
  lookupCosAngle_.clear();
  float angle_step_size = pcl::deg2rad(360.0f)/float(descriptor_size_);
  for (int descriptor_value_idx=0; descriptor_value_idx<descriptor_size_; ++descriptor_value_idx)
  {
    float angle = descriptor_value_idx*angle_step_size;
    lookupSinAngle_.push_back(sinf(angle));
    lookupCosAngle_.push_back(cosf(angle));
  }
}

void MyPcl::NarfssKeypoint::setDescriptorSize(int descriptor_size)
{
  clearData();
  descriptor_size_ = descriptor_size;
  updateLookupTables();
}

void MyPcl::NarfssKeypoint::setDefaultParameterSet (MyPcl::NarfssKeypoint::DefaultParameterSet default_parameter_set, float range_dependent_noise)
{
  switch (default_parameter_set)
  {
    case FAST_PARAMETER_SET:
      std::cout << "Setting runtime optimized parameter values.\n";
      setDescriptorSize(8);
      parameters_.interest_image_blur_radius = 1;
      parameters_.optimal_range_image_patch_size = 8;
      parameters_.range_image_blur_radius = 0;
      break;
    case NORMAL_PARAMETER_SET:
      std::cout << "Setting runtime/quality compromise parameter values.\n";
      setDescriptorSize(16);
      parameters_.interest_image_blur_radius = 1;
      parameters_.optimal_range_image_patch_size = 12;
      parameters_.range_image_blur_radius = 1;
      break;
    case HIGH_QUALITY_PARAMETER_SET:
      std::cout << "Setting quality optimized parameter values.\n";
      setDescriptorSize(36);
      parameters_.interest_image_blur_radius = 2;
      parameters_.optimal_range_image_patch_size = 15;
      parameters_.range_image_blur_radius = 1;
      break;
  }
  parameters_.min_interest_value = 0.04f / float(parameters_.interest_image_blur_radius+1);
  parameters_.range_dependent_noise = range_dependent_noise;
}
