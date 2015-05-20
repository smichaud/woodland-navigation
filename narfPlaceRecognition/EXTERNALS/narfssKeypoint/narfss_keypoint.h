/* \author Bastian Steder - steder@informatik.uni-freiburg.de */

#ifndef NARFSS_KEYPOINT_H_
#define NARFSS_KEYPOINT_H_

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/keypoint.h>
#include <pcl/common/angles.h>
#include <pcl/range_image/range_image.h>

// Forward declarations
namespace pcl {
  class RangeImage;
}

namespace MyPcl {

/** \brief @b NARFSS stands for 'Normal Aligned Radial Feature Self Similarity'.
 *            It computes keypoints based on a self similarity analysis of NARF-descriptors.
  * \author Bastian Steder
  * \ingroup keypoints
  */
class PCL_EXPORTS NarfssKeypoint : public pcl::Keypoint<pcl::PointWithRange, int>
{
  public:
    // =====TYPEDEFS=====
    typedef pcl::Keypoint<pcl::PointWithRange, int> BaseClass;
    typedef BaseClass::PointCloudOut PointCloudOut;
    
    // =====ENUMS=====
    enum DefaultParameterSet {
      FAST_PARAMETER_SET,         // Runtime optimized parameters
      NORMAL_PARAMETER_SET,       // Compromise between runtime and quality
      HIGH_QUALITY_PARAMETER_SET  // Quality optimized parameters
    };
    
    // =====PUBLIC STRUCTS=====
    //! Parameters used in this class
    struct Parameters
    {
      Parameters() : support_size(-1.0f), max_no_of_keypoints(-1), distribute_keypoints_uniformly(true),
                     min_dist_between_keypoints_factor(0.0f), min_interest_value(0.02f), do_non_maximum_suppression(true),
                     max_no_of_threads(1), optimal_range_image_patch_size(12),
                     optimal_neighbor_distance_factor(0.25), interest_image_blur_radius(1),
                     minimum_range(-1.0f), maximum_range(-1.0f), range_dependent_noise(0.0f), range_image_blur_radius(1),
                     prevent_keypoints_in_background(true), use_max_descr_dist_instead_of_min(false)
      {}
      
      float support_size;                       //!< This defines the area 'covered' by an interest point (in meters)
      int max_no_of_keypoints;                  //!< The maximum number of interest points that will be returned
      bool distribute_keypoints_uniformly;      /**< This only has an effect, if max_no_of_keypoints is greater than zero.
                                                     Then, out of all the potential keypoints it will select those that cover
                                                     most of the scene, while still preferring the ones with the highes interest value. */
      float min_dist_between_keypoints_factor;  /**< Minimum distance between maximas
                                                  *  (this is a factor for support_size, i.e. the distance is
                                                  *  min_dist_between_keypoints_factor*support_size) */
      float min_interest_value;                 //!< The minimum value to consider a point as an interest point
      bool do_non_maximum_suppression;          /**< If this is set to false there will be much more points
                                                  *  (can be used to spread points over the whole scene
                                                  *  (combined with a low min_interest_value)) */
      int max_no_of_threads;                    //!< The maximum number of threads this code is allowed to use with OPENMP
      int optimal_range_image_patch_size;       /**< The size (in pixels) of the image patches from which the interest value
                                                  *  should be computed. This influences, which range image is selected from
                                                  *  the scale space to compute the interest value of a pixel at a certain
                                                  *  distance. */
      float optimal_neighbor_distance_factor;   /**< Optimal distance of the neighbors that are used to calculate the self similarity
                                                  *  (this is a factor for support_size, i.e. the distance is
                                                  *  optimal_neighbor_distance_factor*support_size) */
      int interest_image_blur_radius;           /**< Pixel radius used to blur the interest image. */
      float minimum_range;                      /**< Minimum range below which no keypoints are extracted. */
      float maximum_range;                      /**< Maximum range above which no keypoints are extracted. */
      float range_dependent_noise;              /**< Noise that increases with the range, e.g., for stereo or openni devices like
                                                  *  the Microsoft Kinect or the Asus Xtion Pro. E.g., 0.1 means that the measurement error
                                                  *  increases by 10cm every meter. This value will decrease the number of false keypoints
                                                  *  in far away areas with high noise. The noise for openni devices should be around 0.1 */
      float range_image_blur_radius;            //!< During the descriptor calculation, this value is used to blur the range image and suppress noise
      bool prevent_keypoints_in_background;     /**< If active, this is supposed to move keypoints from the
                                                  *  background to the foreground at object borders */
      bool use_max_descr_dist_instead_of_min;    /**< This parameter decides if the interes value of a point is calculated as the minimum
                                                      or maximum descriptor distance to its neighbors. False will only create keypoints,
                                                      e.g. in corners, whereas true will also create keypoints on straight edges. */

    };
    
    // =====CONSTRUCTOR & DESTRUCTOR=====
    NarfssKeypoint (const pcl::RangeImage* range_image=NULL, float support_size=-1.0f);
    ~NarfssKeypoint ();
    
    // =====PUBLIC METHODS=====
    //! Erase all data calculated for the current range image
    void
      clearData ();
    
    //! Set the RangeImage member of the RangeImageBorderExtractor
    void
      setRangeImage (const pcl::RangeImage* range_image);
    
    /** Set default one of the pre-defined parameter sets (see enum DefaultParameterSet above)
     *  Additionally, the second value sets the corresponding parameter value (see parameters above) */
    void
      setDefaultParameterSet (DefaultParameterSet default_parameter_set, float range_dependent_noise=0.0f);
    
    //! Extract interest value per image point
    const std::vector<float>&
      getInterestImage () { calculateInterestImage(); return *interest_image_scale_space_[0]; }
    
    //! Extract interest points from the interest image
    const ::pcl::PointCloud<pcl::InterestPoint>&
      getKeypoints (bool no_calc=false) { if(!no_calc) calculateKeypoints(); return keypoints_; }
    
    //! Getter for the parameter struct
    Parameters&
      getParameters () { return parameters_;}
    
    //! Getter for the descriptor size
    int
      getDescriptorSize () { return descriptor_size_;}
    
    //! Getter for the range image
    const pcl::RangeImage&
      getRangeImage ();
    
    //! Overwrite the compute function of the base class
    virtual void
      compute (PointCloudOut& output);
    
    //! Change the size of the descriptors used to calculate the self similarity
    void setDescriptorSize(int descriptor_size=16);
    
    /** Extract a descriptor in the given image point in the range image.
      * This function is only public for debugging or visualization purposes.
      * If you do not know what this is about, then just leave it be. */
    bool
      extractDescriptor(const pcl::RangeImage& range_image, int x, int y, float* descriptor,
                        float& minimum_distance_value, int* neighbor_indices, int* descriptor_end_indices=NULL) const;
    
    /** Extract the interest value at a certain point index at a certain scale.
      * This function is only public for debugging or visualization purposes.
      * If you do not know what this is about, then just leave it be. */
    float
      extractInterestValue(int point_index, int scale_space_idx) const;
    
    //! Activate debug mode - provides more terminal output
    void setDebug(bool debug) { debug_=debug; }
    
    //! Getter for internal data - for debugging/visualization purposes only
    const std::vector<const pcl::RangeImage*>& getRangeImageScaleSpace() const { return range_image_scale_space_; }
    //! Getter for internal data - for debugging/visualization purposes only
    const std::vector<float>& getScaleSpaceUsageRanges() const { return scale_space_usage_ranges_; }
    //! Getter for internal data - for debugging/visualization purposes only
    const std::vector<std::vector<float>*>& getDescriptorsScaleSpace() const { return descriptors_scale_space_; }
    //! Getter for internal data - for debugging/visualization purposes only
    const std::vector<std::vector<int>*>& getNeighborIndicesScaleSpace() const { return neighbor_indices_scale_space_; }
    //! Getter for internal data - for debugging/visualization purposes only
    const std::vector<std::vector<float>* >& getInterestImageScaleSpace() const { return interest_image_scale_space_; }
    //! Getter for internal data - for debugging/visualization purposes only
    const std::vector<std::vector<char>* >& getCalculatedAtThisScaleScaleSpace() const { return calculated_at_this_scale_scale_space_; }
    //! Getter for internal data - for debugging/visualization purposes only
    const std::vector<std::vector<int>*>& getKeypointIndicesScaleSpace() const { return keypoint_indices_scale_space_; }
    
  protected:
    // =====PROTECTED METHODS=====
    static inline float
      getDescriptorDistance(const float* descriptor, const float* descriptor2, int descriptor_size);
    
    inline void
      blurDescriptor(float* descriptor, int descriptor_size) const;
    
    void
      extractDescriptors();
    
    void
      blurInterestImage (int scale_space_idx);
    
    void
      filterBorderNoise (int scale_space_idx);
    
    void updateLookupTables ();
    
    void
      calculateRangeImageScaleSpace ();
    
    void
      calculateInterestImage ();
    
    void
      calculateKeypoints ();
    
    //! Detect key points
    virtual void
      detectKeypoints (PointCloudOut& output);
    
    // =====PROTECTED MEMBER VARIABLES=====
    using BaseClass::name_;
    Parameters parameters_;
    ::pcl::PointCloud<pcl::InterestPoint> keypoints_;
    std::vector<const pcl::RangeImage*> range_image_scale_space_;
    std::vector<float> scale_space_usage_ranges_;
    std::vector<std::vector<float>*> descriptors_scale_space_;
    std::vector<std::vector<float>*> minimum_distance_values_scale_space_;
    std::vector<std::vector<int>*> neighbor_indices_scale_space_;
    std::vector<std::vector<float>*> interest_image_scale_space_;
    std::vector<std::vector<char>*> calculated_at_this_scale_scale_space_;
    std::vector<std::vector<int>*> keypoint_indices_scale_space_;
    std::vector<int> original_scale_keypoint_indices_;
    int descriptor_size_;
    std::vector<float> lookupSinAngle_, lookupCosAngle_;
    
    bool debug_;
};

}  // end namespace pcl

#include "narfss_keypoint.hpp"

#endif  //#ifndef NARFSS_KEYPOINT_H_
