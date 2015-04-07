#ifndef SCAN_DATABASE_H
#define SCAN_DATABASE_H

#include <vector>
#include <kdtree++/kdtree.hpp>

#include <pcl/range_image/range_image.h>
//#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/narf.h>
#include <pcl/correspondence.h>
#undef MEASURE_FUNCTION_TIME


/////FORWARD DECLARATIONS/////
namespace pcl {
  template <typename T1, typename T2>
  class KdTreeFLANN;
}
#include "EXTERNALS/spm/spmBase/narfInstance.h"
#include "EXTERNALS/spm/spmBase/spModel.h"
#include "EXTERNALS/narfssKeypoint/narfss_keypoint.h"

/**
 * \brief Structure to save a 3D scan in form of a range image, and connected data
 **/
class ScanDatabaseElement {
  public:
    //-----STATIC MEMBERS-----
    static bool useRotationInvariance;
    //-----TYPEDEFS & STRUCTS-----
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloudType;
    typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > VectorOfEigenVector3f;
    struct EigenVector3fBracketAccessor {
      typedef float result_type;
      result_type operator() (const Eigen::Vector3f& p, const size_t i) const { return p[i]; }
    };
    typedef KDTree::KDTree<3, Eigen::Vector3f, EigenVector3fBracketAccessor> KDTreeForVector3f;
    struct ValidationPoints : public VectorOfEigenVector3f {
      VectorOfEigenVector3f normals;
    };
    typedef std::map<std::string, Eigen::Isometry3f, std::less<std::string>, Eigen::aligned_allocator<std::pair<std::string, Eigen::Isometry3f> > > MapStringToIsometry3f;
    
    //-----CONSTRUCTOR&DESTRUCTOR-----
    /** Constructor */
    ScanDatabaseElement();
    //! Copy Constructor
    ScanDatabaseElement(const ScanDatabaseElement& other);
    /** Destructor */
    virtual ~ScanDatabaseElement();

        
    // -----Operators-----
    //! Assignment operator
    const ScanDatabaseElement& operator=(const ScanDatabaseElement& other);
    
    //-----METHODS-----
    
    void reset();
    void resetClouds();
    void resetRangeImage();
    void resetFeatures();
    void resetDictionaryFeatures();
    void resetGlobalFeatures();
    void resetValidationPoints();
    void loadPointCloud();
    void createRangeImage(float angularResolution, float noiseLevel,
                          pcl::RangeImage::CoordinateFrame coordinateFrame=pcl::RangeImage::CAMERA_FRAME, float maximumRange=-1);
    void saveRangeImage() const;
    void loadRangeImage();
    /** Extract features from the scan.
      * \param doNotResetKeypointDetector this can be used to save runtime. If the keyPointDetector is not reset,
      *        its intermediate results can be used for the validation point calculation.
      */
    void extractNARFs(float supportSize, int descriptorSize, bool rotationInvariant,
                      MyPcl::NarfssKeypoint& keyPointDetector, bool doNotResetKeypointDetector=false);
    //void extractNARFs(float supportSize, int descriptorSize, bool rotationInvariant, pcl::NarfKeypoint& keyPointDetector);
    /** Extract validation points from the scan.
      * \param reuseKeypointDetector this can be used to save runtime. If the keyPointDetector is not reset after feature extraction,
      *                              its intermediate results can be reused here.
      */
    void extractValidationPoints(int noOfPoints, MyPcl::NarfssKeypoint& keyPointDetector, bool reuseKeypointDetector=false);
    //void extractValidationPoints(int noOfPoints, pcl::NarfKeypoint& keyPointDetector, float minSurfaceChange);
    void calculateNormalsForValidationPoints();
    void extractDictionaryNARFs(float supportSize, int descriptorSize, bool rotationInvariant, MyPcl::NarfssKeypoint& keyPointDetector);
    void getMatches(const pcl::Narf& feature, int featureId, float maxDescriptorDistance,
                    float minDistanceBetweenMatches, pcl::PointCorrespondences6DVector& featureMatches) const;
    void getMatches(const ScanDatabaseElement& scan, float maxDescriptorDistance,
                    float minDistanceBetweenMatches, pcl::PointCorrespondences6DVector& featureMatches) const;
    
    void extractGlobalFeatures(float maximumRangeInDataset);
    
    boost::shared_ptr<pcl::RangeImage> getRangeImageAsBoostSharedPtr();
    
    //void updateKdTree();
    
    void loadNARFs();
    void loadDictionaryNARFs();
    void saveDictionaryNARFs() const;
    void saveNARFs() const;
    void loadValidationPoints();
    void saveValidationPoints() const;
    void loadInfoFile();
    void saveInfoFile() const;
    void loadDictionaryHistogram();
    void saveDictionaryHistogram() const;
    
    float getHistogramDistance(const std::vector<double>& histogram) const;
    
    //-----PUBLIC VARIABLES-----
    std::string pointCloudFileName;
    pcl::PCLPointCloud2 pointCloudData, pointCloudDataFarRanges;
    PointCloudType pointCloud, pointCloudFarRanges;
    pcl::RangeImage rangeImage;
    std::vector<pcl::Narf*> features;
    std::vector<pcl::Narf*> featuresForDictionary;
    ValidationPoints validationPoints;
    float selfSimilarity;
    MapStringToIsometry3f knownPoses;
    std::map<std::string, std::string> unknownInfoFileElements;
    std::vector<double> dictionaryHistogram;
    std::vector<pcl::Narf*> narfsForCompleteScan;
    //pcl::KdTreeFLANN<pcl::PointWithRange> kdTreeForRangeImage;
    //KDTreeForVector3f kdTreeForRangeImage;
    
  protected:
    //! Create a deep copy of other
    void deepCopy(const ScanDatabaseElement& other);
    
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * \brief Structure to save a list of ScanDatabaseElements
 **/
class ScanDatabase : public std::vector<ScanDatabaseElement*> {
  public:
    //-----TYPEDEFS-----
    typedef std::vector<ScanDatabaseElement*> BaseClass;
    
    //-----CONSTRUCTOR&DESTRUCTOR-----
    /** Constructor */
    ScanDatabase();
    ~ScanDatabase();
    
    //-----PUBLIC METHODS-----
    virtual void clear();
    void load(const std::string& directoryName);
    void saveConfigFile();
    void getMatches(const ScanDatabaseElement& scan, float maxDescriptorDistance, float minDistanceBetweenMatches,
                    std::vector<pcl::PointCorrespondences6DVector>& featureMatchesPerScan,
                    bool useKdtree=false, float maxDescriptorDistanceKdtree=-1.0f) const;
    void getMatches(const pcl::Narf& feature, int featureId, float maxDescriptorDistance, float minDistanceBetweenMatches,
                    std::vector<pcl::PointCorrespondences6DVector>& featureMatchesPerScan,
                    bool useKdtree=false, float maxDescriptorDistanceKdtree=-1.0f) const;
    void orderScansRegardingHistogramSimilarity(const std::vector<double>& histogram, int maxNoOfScans, std::vector<int>& scanIndices) const;
    void orderScansRegardingGlobalFeatureSimilarity(const ScanDatabaseElement& scan, int maxNoOfScans, std::vector<int>& scanIndices) const;
    
    //-----PUBLIC VARIABLES-----
    std::string name;
    std::string databaseDirectory;
    std::map<std::string, std::string> configFileParams;
    
    //VimripList combinedFeatureList;

  protected:
    //-----STRUCTS-----
    struct FeatureSource
    {
      FeatureSource(int scanIndex, int featureIndex) : scanIndex(scanIndex), featureIndex(featureIndex) {}
      int scanIndex, featureIndex;
    };
    pcl::KdTreeFLANN<pcl::Narf*>* kdTreeForFeatures_;
    std::vector<FeatureSource> featureSources_;
};
#endif
