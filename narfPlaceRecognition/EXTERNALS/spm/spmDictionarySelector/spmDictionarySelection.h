#include "../spmBase/spModel.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <vector>
#include <omp.h>

struct PointLikelihood{
    double value;
    bool recalculate;
    double sigma;
    double sqrSigma;
};


struct FeatureAssignment{
   int key1;
   int key2;
   double descriptorDistance;
   double likelihood;
   double likelihoodBackup;
   double sigma;
   bool calculated;
   inline FeatureAssignment(){
      likelihood = 0.0;
      likelihoodBackup = 0.0;
      sigma = 0.0;
      calculated = false;
   }

   inline FeatureAssignment(const FeatureAssignment& other){
      key1 = other.key1;
      key2 = other.key2;
      descriptorDistance = other.descriptorDistance;
      likelihood = other.likelihood;
      likelihoodBackup = other.likelihoodBackup;
      calculated = other.calculated;
      sigma = other.sigma;
   }

  FeatureAssignment& operator=(const FeatureAssignment& other) {
      key1 = other.key1;
      key2 = other.key2;
      descriptorDistance = other.descriptorDistance;
      likelihood = other.likelihood;
      likelihoodBackup = other.likelihoodBackup;
      calculated = other.calculated;
      sigma = other.sigma;
      return *this;
  }
};

struct SelectionParams{
  int maxIterations;
  int iterationsWithoutChange;
  double evaluationResolution;
  double sigma;
};

class SPMDictionarySelector{

   public:
   SPMDictionarySelector();
   ~SPMDictionarySelector();

   bool showTimings;
   bool showDebug;

   /** Dictionary selection using iterative BIC procedure */
   void clusterDictionary(SPModel& model, pcl::PointCloud<pcl::PointXYZ>& data, SelectionParams& params);

   /** dictionary management stuff */
   void sampleDeleteDictionary(SPModel& model, NarfWord*& deletedWord);
   void sampleInsertDictionary(SPModel& model, NarfWord*& addedWord);
   void sampleReplaceDictionary(SPModel& model);
   bool wordInDictionary(SPModel& SPModel, unsigned int& wordId);

   double calculateBIC(pcl::PointCloud<pcl::PointXYZ>& model, pcl::PointCloud<pcl::PointXYZ>& data, double& sigma, int numOfInstances);
   double calculateBICOnWords(SPModel& model, unsigned int& dataSize, double& sigma);
   double calculateLikelihood(pcl::PointCloud<pcl::PointXYZ> & model, pcl::PointCloud<pcl::PointXYZ> & data, double& sigma);
 
   void initializeChangedRegions(pcl::PointCloud<pcl::PointXYZ>& data);
   void markChangedRegions(SPModel& model, pcl::PointCloud<pcl::PointXYZ>& data);
   NarfWord* getNarfWordForInstance(NarfInstance& instance);
   NarfWord* getNarfWordForInstance(NarfInstance& instance, std::vector<NarfWord* >& dictionary);

   void writeBinary(std::ostream &file) const;
   void writeBinary(char* fileName) const;

   void readBinary(std::istream &file, SPModel& model);
   void readBinary(const char* fileName, SPModel& model);

   void clusterDictionaryWithDescriptorDistance(SPModel& model, double treshold);
   void fillModelPointCloudForDictionary(SPModel& model, pcl::PointCloud<pcl::PointXYZ>& cloud, std::vector<NarfWord*>& dictionary, double& resolution);

   void clusterDictionaryWithkMeans(SPModel& model, int numOfClusters, int attempts);

   void optimizeInstancePoses(SPModel& model, pcl::PointCloud<pcl::PointXYZ>& cloud);

   std::vector<NarfWord* > _dictionary;
   std::vector<NarfWord* > _dictionaryBackup;

   double _bic;
   double _lastbic;

   std::vector<bool> _reCalcLikelihood;
   std::vector<double> _likelihood; 
   std::vector<double> _likelihoodBackup; 
   std::map<unsigned int, std::map<unsigned int, double> > likelihoodForInstances;
};
