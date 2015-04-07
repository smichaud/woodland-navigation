#ifndef SP_MODEL_H
#define SP_MODEL_H

#include <iostream>
#include <sstream>
#include <vector>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/features/narf.h"
#include "pcl/kdtree/kdtree_flann.h"

//forward declerations
struct NarfInstance;
struct NarfWord;
class SPMDictionarySelector;

class SPModel{
   public:
   typedef pcl::PointCloud<pcl::PointXYZ> pclPointCloud;
   typedef std::vector<pcl::Narf*> NarfList;
   friend class SPMDictionarySelector;

   SPModel();
   ~SPModel();

   void clear();
   void clearInstances();
   // ---------------------------------
   // Instances
   // ---------------------------------
   //! adds instance and linked word to dictionary
   void addInstanceAndWord(NarfInstance& instance);
   //! adds instance
   void addInstance(NarfInstance& instance);

   //! deletes an instance
   void deleteInstance(NarfInstance& instance);
   //! deletes all marked instances
   void deleteMarkedInstances();

   // ---------------------------------
   // Words
   // ---------------------------------
   //! adds word to dictionary
   void addWord(NarfWord*& instance);

   //! deletes word from dictionary and frees pointer
   void deleteWord(NarfWord*& word);
   //! deletes all marked words from dictionary and frees pointers
   void deleteMarkedWords();

   // ---------------------------------
   // Point cloud stuff
   // ---------------------------------
   //! constructs the model point cloud with a minimum resolution of 0.01
   pclPointCloud getModelPointCloud();
   //! constructs the model point cloud with a minimum resolution
   pclPointCloud getModelPointCloud(double& resolution);
   //! returns model point cloud constructed from the given dictionary
   pclPointCloud getModelPointCloudForDictionary(std::vector<NarfWord*>& dictionary, double& resolution);
   //! constructs the model point cloud with the provided dictionary
   void fillModelPointCloudForDictionary(SPModel::pclPointCloud& cloud, std::vector<NarfWord*>& dictionary, double& resolution);
   //! returns a point cloud that includes all instances
   pclPointCloud getInstancePointCloud();
   //! adds points into scanPointCloud for every instance
   void instancesToPointCloud(std::vector<NarfInstance>& scanInstances, pclPointCloud& scanPointCloud);

   // ---------------------------------
   // Instance to Word assignments stuff
   // ---------------------------------
   enum InstanceWordAssignmentStrategie {DescriptorDistance, MaximumLikelihood};
   //! returns map between scanInstance ids and word ids from the given dictionary. Computed based on the given strategie.
   std::map<int, int> getInstance2WordMap(std::vector<NarfInstance>& scanInstances, std::vector<NarfWord* >& dictionary, InstanceWordAssignmentStrategie strategie);
   //! returns id of best fitting word for given instance. Calculated based on the descriptor distance.
   int getNarfWordIdForInstance(NarfInstance& instance, std::vector<NarfWord* >& dictionary);
   //! returns id of best fitting word for given instance. Calculated based on the maximum liklihood word point cloud.
   int getNarfWordIdForInstanceMaximumLikelihood(NarfInstance& instance, std::vector<NarfWord* >& dictionary);

   protected:
   //! converts PointXYZ in cell index for given resolution (Needed by getModelPointCloud)
   void pose2mapIndex(pcl::PointXYZ& point, Eigen::Vector3i& index, double& resolution);

   // ---------------------------------
   // I/O stuff
   // ---------------------------------
   protected:
   //! sets instance to word pointers and fills wordMap
   void linkWordsAndInstances();
   public:
   void writeBinary(std::ostream &file) const;
   void readBinary(std::istream &file);

   // ---------------------------------
   // Dictionary stuff
   // ---------------------------------
   //! Overwrites the existing dictionary and updates instances accordingly
   void importDictionary(std::vector<NarfWord*>& dictionary);
   //! Sorts dictionary vector according to the occurance frequency in the instances
   void sortDictionary();
   //! Counts word occurances in the model. Result will be stored in the count member of every word.
   void countWordOccurances();
   //! Deletes all words that do not occure in the model 
   void deleteUnusedWords();
   //! outputs statistics 
   void stats();

   // ---------------------------------
   // Instance stuff
   // ---------------------------------
   void sparseInstances(double support_size);

   // ---------------------------------
   // Member variables
   // ---------------------------------
   inline std::vector<NarfInstance>& instances(){ return _instances; };
   inline std::vector<NarfWord*>& dictionary(){ return _dictionary; };
   inline std::map<int, NarfWord*>& wordMap(){ return _wordMap; };

   protected:
   std::vector<NarfInstance>   _instances;
   std::vector<NarfWord*>      _dictionary;
   std::map<int, NarfWord*>    _wordMap;
   unsigned int                _maxWordId;

   public:
   std::vector<double> getWordHistogram(std::vector<NarfWord* >& dictionary);
};

#endif 
 
 