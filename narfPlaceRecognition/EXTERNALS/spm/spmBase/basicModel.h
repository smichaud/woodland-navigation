#ifndef BASIC_MODEL_H
#define BASIC_MODEL_H

#include <iostream>
#include <sstream>
#include <vector>
#include <limits>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/features/narf.h"
#include "pcl/kdtree/kdtree_flann.h"

#include "basicWord.h"
#include "basicInstance.h"

#include "basics/timeutil.h"

namespace Ais3dTools{
  
  //forward declerations
  class SPMDictionarySelector;

  template<typename DescriptionType>
  class BasicModelT{
    public:
    typedef pcl::PointWithRange 		        PointType;
    typedef typename pcl::PointCloud<PointType>     pclPointCloud;
    typedef std::vector<pcl::Narf*> NarfList;
    friend class SPMDictionarySelector;

    BasicModelT();
    ~BasicModelT();

    void clear();
    void clearInstances();
    // ---------------------------------
    // Instances
    // ---------------------------------
    //! adds instance and linked word to dictionary
    void addInstanceAndWord(BasicInstanceT<DescriptionType>& instance);
    //! adds instance
    void addInstance(BasicInstanceT<DescriptionType>& instance);

  //    //! deletes an instance
  //    void deleteInstance(BasicInstanceT<DescriptionType>& instance);
  //    //! deletes all marked instances
  //    void deleteMarkedInstances();

    // ---------------------------------
    // Words
    // ---------------------------------
    //! adds word to dictionary
    void addWord(BasicWordT<DescriptionType>*& instance);

    //! deletes word from dictionary and frees pointer
    void deleteWord(BasicWordT<DescriptionType>*& word);
    //! deletes all marked words from dictionary and frees pointers
    void deleteMarkedWords();

    // ---------------------------------
    // Point cloud stuff
    // ---------------------------------
  //    //! constructs the model point cloud with a minimum resolution of 0.01
  //    pclPointCloud getModelPointCloud();
  //    //! constructs the model point cloud with a minimum resolution 
  void getModelPointCloud(pclPointCloud& cloud, double resolution);
  //    //! returns model point cloud constructed from the given dictionary
  //    pclPointCloud getModelPointCloudForDictionary(std::vector<BasicWordT<DescriptionType>*>& dictionary, double& resolution);
  //    //! constructs the model point cloud with the provided dictionary
  //    void fillModelPointCloudForDictionary(SPModel::pclPointCloud& cloud, std::vector<BasicWordT<DescriptionType>*>& dictionary, double& resolution);
  //    //! returns a point cloud that includes all instances
  //    pclPointCloud getInstancePointCloud();
  //    //! adds points into scanPointCloud for every instance
  //    void instancesToPointCloud(std::vector<BasicInstanceT<DescriptionType>>& scanInstances, pclPointCloud& scanPointCloud);

    // ---------------------------------
    // Instance to Word assignments stuff
    // ---------------------------------
  //    enum InstanceWordAssignmentStrategie {DescriptorDistance, MaximumLikelihood};
  //    //! returns map between scanInstance ids and word ids from the given dictionary. Computed based on the given strategie.
  //    std::map<int, int> getInstance2WordMap(std::vector<BasicInstanceT<DescriptionType>>& scanInstances, std::vector<BasicWordT<DescriptionType>* >& dictionary, InstanceWordAssignmentStrategie strategie);
  //    //! returns id of best fitting word for given instance. Calculated based on the descriptor distance.
  //    int getBasicWordT<DescriptionType>IdForInstance(BasicInstanceT<DescriptionType>& instance, std::vector<BasicWordT<DescriptionType>* >& dictionary);
  //    //! returns id of best fitting word for given instance. Calculated based on the maximum liklihood word point cloud.
  //    int getBasicWordT<DescriptionType>IdForInstanceMaximumLikelihood(BasicInstanceT<DescriptionType>& instance, std::vector<BasicWordT<DescriptionType>* >& dictionary);

    protected:
    //! converts PointXYZ in cell index for given resolution (Needed by getModelPointCloud)
    void pose2mapIndex(PointType& point, Eigen::Vector3i& index, double& resolution);

    // ---------------------------------
    // I/O stuff
    // ---------------------------------
    protected:
    //! sets instance to word pointers and fills wordMap
    void linkWordsAndInstances();
    public:
    void writeBinary(std::ostream &file) const;
    void readBinary(std::istream &file);

    void exportWordsAscii(std::ostream &file) const;
    void exportDefinedBitMaskAscii(std::ostream &file) const;
    void evaluateBitMaskAscii(std::istream &dictFile, std::istream &gammeFile, int wordPixelWidth);
    
    ///loads matrices for dictionary and linear combinations for every instance
    void importDictionaryAscii(std::istream &dictFile, std::istream &gammeFile, int wordPixelWidth);
    void evaluateDictionaryAscii(std::istream &dictFile, std::istream &gammeFile, int wordPixelWidth);
    
    void exportInstanceOrientationsAscii(std::ostream &file) const;
    void importInstanceOrientationsAscii(std::istream &file);
    // ---------------------------------
    // Dictionary stuff
    // ---------------------------------
  //    //! Overwrites the existing dictionary and updates instances accordingly
  //    void importDictionary(std::vector<BasicWordT<DescriptionType>*>& dictionary);
  //    //! Sorts dictionary vector according to the occurance frequency in the instances
  //    void sortDictionary();
  //    //! Counts word occurances in the model. Result will be stored in the count member of every word.
  //    void countWordOccurances();
  //    //! Deletes all words that do not occure in the model 
  //    void deleteUnusedWords();
  //    //! outputs statistics 
    void stats();

    // ---------------------------------
    // Instance stuff
    // ---------------------------------
    void sparseInstances();

    // ---------------------------------
    // Member variables
    // ---------------------------------
    inline std::vector<BasicInstanceT<DescriptionType> >& instances(){ return _instances; };
    inline std::vector<BasicWordT<DescriptionType>* >&    dictionary(){ return _dictionary; };
    inline std::map<int, BasicWordT<DescriptionType>* >&  wordMap(){ return _wordMap; };

    protected:
    std::vector<BasicInstanceT<DescriptionType> >   _instances;
    std::vector<BasicWordT<DescriptionType>* >      _dictionary;
    std::map<int, BasicWordT<DescriptionType>* >    _wordMap;
    unsigned int                			 _maxWordId;
    float 					 _wordZOffset;

    public:
    std::vector<double> getWordHistogram(std::vector<BasicWordT<DescriptionType>* >& dictionary);
  };

#include "basicModel.hpp"
  
} //end namespace
#endif 
 
 