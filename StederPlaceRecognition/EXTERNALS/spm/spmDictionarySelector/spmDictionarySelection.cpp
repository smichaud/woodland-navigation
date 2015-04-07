#include "spmDictionarySelection.h"
#include "../spmBase/narfWord.h"
#include "../spmBase/narfInstance.h"
#include "../spmBase/likelihoodModels.h"
//#include "projects/modelLearning/model/threadTools/rwlock.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <boost/make_shared.hpp>
#include <omp.h>
#include "basics/timeutil.h"
#include <iostream>
#include <fstream>
#include "../kmeans++/KMeans.h"

using namespace std;
using namespace pcl;
using namespace Ais3dTools;

SPMDictionarySelector::SPMDictionarySelector(){
  _lastbic = 10e30;
  _bic = 10e30;
}

SPMDictionarySelector::~SPMDictionarySelector(){

}


void SPMDictionarySelector::clusterDictionary(SPModel& model, pcl::PointCloud<pcl::PointXYZ>& data, SelectionParams& params)
{
  //calculate data Size of original data
    unsigned int dataSize = 0;
    for(unsigned int i = 0; i < model._instances.size(); i++){
      dataSize += model._instances[i].word->pointCloud.size();
    }
    dataSize = dataSize * 3;

  int iterationsWithoutChange = 0;
  //initializeChangedRegions(data);

  //cout parameters
  cout << params.maxIterations << " ";
  cout << params.iterationsWithoutChange << " ";
  cout << params.evaluationResolution << " ";
  cout << params.sigma << endl;

  for(int i = 0; i < params.maxIterations; i++){
    double iterationStartTime = get_time();
    int step = rand() % 5;

    double time = get_time();
    if(step <= 2 || _dictionary.size() == 0) 
    {
      step = 0;
      NarfWord* addedWord = 0;
      sampleInsertDictionary(model, addedWord);
    }
    
    if(step == 3) //DELETE
    {
      NarfWord* deletedWord = 0;
      sampleDeleteDictionary(model, deletedWord);
    }

    if(step == 4) //REPLACE
    {
      sampleReplaceDictionary(model);
    }
//     cerr << "Dictionary sampling took: "<< (get_time() - iterationStartTime) << "s " << std::endl;

    time = get_time();

    //Evaluate current _dictionary
    pcl::PointCloud<pcl::PointXYZ> modelCloud;
    if(_dictionary.size() == 0)
      continue; 

    model.fillModelPointCloudForDictionary(modelCloud, _dictionary, params.evaluationResolution);
//     cerr << "Model construction took: "<< (get_time() - iterationStartTime) << "s " << std::endl;

    //fillModelPointCloudForDictionary(model, modelCloud, _dictionary, params.evaluationResolution);

    if(_dictionaryBackup.size() == 0)
      _dictionaryBackup = _dictionary;
    //markChangedRegions(model, data);

    time = get_time();
    _bic = calculateBIC(modelCloud, data, params.sigma, model._instances.size());
    double bicTime = (get_time() - time) * 1000.0;
    //_bic = calculateBICOnWords(model, dataSize);


    cerr << "Iteration: " << PVAR(i) << " \t| ";
    if(_bic < _lastbic){
        cerr << " success";
        cout << i << " " << _dictionaryBackup.size() << " " << _lastbic << " " << _bic << " " << get_time() - iterationStartTime << endl;
        _lastbic = _bic;
        iterationsWithoutChange = 0;
      //Backup _dictionary 
      _dictionaryBackup = _dictionary;
      for(size_t k=0; k < _likelihood.size(); ++k)
        _likelihoodBackup[k] = _likelihood[k];
    } else {
      cerr << " reject ";
      _dictionary = _dictionaryBackup;
      for(size_t k=0; k < _likelihood.size(); ++k)
        _likelihood[k] = _likelihoodBackup[k];
      iterationsWithoutChange++;
    }
    cerr << " \t|  "<< PVAR(_bic) << " \t|  " << PVAR(_dictionary.size()) << " \t| "; 
    cerr << "bic took: " << bicTime << "ms " << " \t| "; 
    cerr << "iteration took: " << (get_time() - iterationStartTime)*1000.0 << "ms " << std::endl;


    //Save current result every 20 steps
    if(i % 20 == 0 && i > 0){
      char filename[128];
      sprintf(filename, "iteration%07d.dic", i);
      writeBinary(filename);
    }

    if(iterationsWithoutChange >= params.iterationsWithoutChange){
      break;
    }
  }
  cerr << "Transfer new _dictionary to model" << endl;
  model.importDictionary(_dictionary);
}

void SPMDictionarySelector::sampleInsertDictionary(SPModel& model, NarfWord*& addedWord)
{
  //! Select new word 
  unsigned int newWordIndex = rand() %  model._dictionary.size();

  //!Check if word is in _dictionary
  bool inDictionary = wordInDictionary(model, newWordIndex);
  if(inDictionary || model._dictionary[newWordIndex]->pointCloud.size() < 15){
    sampleInsertDictionary(model, addedWord);
    return;
  } else {
    //!insert selected word
    _dictionary.push_back(model._dictionary[newWordIndex]);
  }
  addedWord = model._dictionary[newWordIndex];
}

void SPMDictionarySelector::sampleDeleteDictionary(SPModel& model, NarfWord*& deletedWord)
{
  (void) model;  // No warning
  (void) deletedWord;  // No warning
  
  //! Select word to reduce
  unsigned int wordId = rand() %   _dictionary.size();
  wordId = _dictionary[wordId]->id;

  vector<NarfWord* > _dictionaryBackup = _dictionary;
  _dictionary.clear();
  //!build up _dictionary without selected word
  for(unsigned int j = 0; j < _dictionaryBackup.size(); ++j)
  {
    if(j != wordId)
    {
      _dictionary.push_back(_dictionaryBackup[j]);
    }
  }
}

void SPMDictionarySelector::sampleReplaceDictionary(SPModel& model)
{
  //! Select word to reduce
  unsigned int outWordIndex = rand() %   _dictionary.size();
  //! Select new word 
  unsigned int inWordIndex = rand() %  model._dictionary.size();

  if(model._dictionary[inWordIndex]->pointCloud.size() < 15){
    sampleReplaceDictionary(model);
    return;
  }
  
  vector<NarfWord* > _dictionaryBackup = _dictionary;
  _dictionary.clear();
  //!build up _dictionary without selected word
  for(unsigned int j = 0; j < _dictionaryBackup.size(); ++j)
  {
    if(j != outWordIndex)
    {
      _dictionary.push_back(_dictionaryBackup[j]);
    }
  }
  _dictionary.push_back(model._dictionary[inWordIndex]);
}

bool SPMDictionarySelector::wordInDictionary(SPModel& model, unsigned int& wordId)
{
  bool in_dictionary = false;
  for(unsigned int j = 0; j < _dictionary.size(); j++)
  {
    if(_dictionary[j]->id == model._dictionary[wordId]->id)
    { 
        in_dictionary = true;
        break;
    }
  }
  return in_dictionary;
}


double SPMDictionarySelector::calculateBIC(pcl::PointCloud<pcl::PointXYZ>& model, pcl::PointCloud<pcl::PointXYZ>& data, double& sigma, int numOfInstances){
    (void) numOfInstances;
    
    /** Model M, Dataset X, N is size of Dataset
      BIC(M) = log L(X,M) - lambda * 0.5 NumParameters * log(N) 
      (Definition taken from: Speaker, Environment and Channel Change Detection... Scott Shaobing Chen */
    double logLikelihood = LikelihoodModels::sphericalGaussianFixSigma(model, data, sigma);

    //calulate the number of parameters for the current model
    unsigned int parameters = 0; //_dictionary.size() + numOfInstances;
    for(unsigned int i = 0; i < _dictionary.size(); i++){
      parameters += _dictionary[i]->pointCloud.size();
    }
    //parameters = parameters * 3;
    //parameters += numOfInstances * 7;

    double bic = -2*(logLikelihood) + parameters * log(data.size()); 
    //cerr << likelihood << " " << parameters << " " << data.size() << std::endl;
    return bic;
}


double SPMDictionarySelector::calculateBICOnWords(SPModel& model, unsigned int& dataSize, double& sigma){
    /** Model M, Dataset X, N is size of Dataset
      BIC(M) = log L(X,M) - lambda * 0.5 NumParameters * log(N) 
      (Definition taken from: Speaker, Environment and Channel Change Detection... Scott Shaobing Chen */

    double likelihood = 0.0;
    #pragma omp parallel for schedule(dynamic, 20) default(shared) reduction(+:likelihood) 
    for(unsigned int j = 0; j < model._instances.size(); j++){
      NarfInstance& instance = model._instances[j];
      NarfWord* wordForInstance = 0;
      //double minDistance = 1e30;
      double minDistance = log(0.0);
      //Get the _dictionary word with the lowest descriptor distance
      for(size_t i = 0; i < _dictionary.size(); ++i){
          NarfWord*& word = _dictionary[i];
          double distance =  calculateLikelihood(word->pointCloud, instance.word->pointCloud, sigma);
          //double distance =  word->feature.getDescriptorDistance(instance.word->feature);
          if(distance > minDistance){
            wordForInstance = word;
            minDistance = distance;
          }
      }
  
      if(wordForInstance == 0){
        cerr << "No word for instance " << instance.fid << std::endl;
        cerr << "This should not happen!!" << endl;
        exit(0);
        //break;
      }
      likelihood += minDistance; //calculateLikelihood(wordForInstance->pointCloud, instance.word->pointCloud);
    }

    //calulate the number of parameters for the current model
    unsigned int parameters = 0;
    for(unsigned int i = 0; i < _dictionary.size(); i++){
      parameters += _dictionary[i]->pointCloud.size();
    }
    parameters = parameters * 3;

    double bic = -2*(likelihood) + parameters * log(dataSize); 
    //cerr << likelihood << " " << parameters << " " << data.size() << std::endl;
    return bic;
}

double SPMDictionarySelector::calculateLikelihood(pcl::PointCloud<pcl::PointXYZ>& model, pcl::PointCloud<pcl::PointXYZ>& data, double& sigma)
{
   double likelihood = LikelihoodModels::sphericalGaussian(model, data, sigma);
   return likelihood;
} 



void SPMDictionarySelector::initializeChangedRegions(pcl::PointCloud<pcl::PointXYZ>& data){
  for(size_t i=0; i < data.size(); ++i){
    _reCalcLikelihood.push_back(true);
    _likelihood.push_back(0.0);
  }
  _likelihoodBackup = _likelihood;
}

void SPMDictionarySelector::markChangedRegions(SPModel& model, pcl::PointCloud<pcl::PointXYZ>& data){
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (data.makeShared ());

  for(unsigned int j = 0; j < model._instances.size(); j++){
    NarfInstance& instance = model._instances[j];
    NarfWord* word = getNarfWordForInstance(instance, _dictionary);
    NarfWord* wordLastStep = getNarfWordForInstance(instance, _dictionaryBackup);
    if(word != wordLastStep){
      Eigen::Vector3f p = instance.pose.translation();
      pcl::PointXYZ currentPoint(p(0), p(1), p(2));
      vector<int> k_indices;
      vector<float> k_distances;
      kdtree.radiusSearch (currentPoint, word->feature.getSurfacePatchWorldSize() * 3.0, k_indices, k_distances);

      for(size_t i=0; i < k_indices.size(); ++i){
        _reCalcLikelihood[ k_indices[i] ] = true;
      }
    }
  }
}

NarfWord* SPMDictionarySelector::getNarfWordForInstance(NarfInstance& instance){
  return getNarfWordForInstance(instance, _dictionary);
}


NarfWord* SPMDictionarySelector::getNarfWordForInstance(NarfInstance& instance, vector<NarfWord* >& dictionary){

  NarfWord* wordForInstance = 0;
  double minDistance = 1e30;

  //Get the _dictionary word with the lowest descriptor distance
  for(size_t i = 0; i < dictionary.size(); ++i){
      NarfWord*& word = dictionary[i];
      double distance =  word->feature.getDescriptorDistance(instance.word->feature);
      if(distance < minDistance){
        wordForInstance = word;
        minDistance = distance;
      }
  }

  if(wordForInstance == 0){
    cerr << "No word for instance " << instance.fid << std::endl;
    cerr << "This should not happen!!" << endl;
    exit(0);
  }
  return wordForInstance;
}

void SPMDictionarySelector::writeBinary(std::ostream &file) const {
  //Features
  file.write((char*)&_lastbic, sizeof(_lastbic));

  unsigned int size = _dictionary.size();
  file.write((char*)&size, sizeof(size));
  for(unsigned int i = 0; i < size; i++){
        unsigned int id = _dictionary[i]->id;
        file.write((char*)&id, sizeof(id));
  }
}

void SPMDictionarySelector::writeBinary(char* fileName) const {
  ofstream file;
  file.open(fileName);
  writeBinary(file);
  file.close();
}


void SPMDictionarySelector::readBinary(std::istream &file, SPModel& model){
  file.read((char*)&_lastbic, sizeof(_lastbic));
  unsigned int size = 0;
  file.read((char*)&size, sizeof(size));
  for(unsigned int i = 0; i < size; i++){
       unsigned int id = 0;
       file.read((char*)&id, sizeof(id));
      _dictionary.push_back(model._wordMap[id]);
  }
}

void SPMDictionarySelector::readBinary(const char* fileName, SPModel& model){
  ifstream file;
  file.open(fileName);
  readBinary(file, model);
  file.close();
}

void SPMDictionarySelector::clusterDictionaryWithDescriptorDistance(SPModel& model, double threshold = 0.1){
   cerr << "Descriptor distance: "<< (threshold) << endl;

   vector<NarfWord* > unassignedWords;
  //preparing dictionary
   for(size_t i = 0; i < model._dictionary.size(); ++i){
     if(model._dictionary[i] != 0)
       unassignedWords.push_back(model._dictionary[i]);
   }

  vector< vector<NarfWord* > >  clusters;
  //Greedy Joint
  //bool run = true;
  while(unassignedWords.size() > 0){
    //float min = 1.0;
    vector<NarfWord* > cluster;
    NarfWord*& referenceWord = unassignedWords[0];
    cluster.push_back(unassignedWords[0]);

    double dist = 1.0;
    vector<int> deleteElements;
    deleteElements.push_back(0);
    for(size_t i = 1; i < unassignedWords.size(); ++i){
       NarfWord*& candidateWord = unassignedWords[i];
       dist = referenceWord->feature.getDescriptorDistance(candidateWord->feature);
       if(dist <= threshold){
          cluster.push_back(candidateWord);
          deleteElements.push_back(i);
       }
    }

    for(size_t i = deleteElements.size() ; i > 0 ; i--){
      unassignedWords.erase(unassignedWords.begin() + deleteElements[i-1]);
      //cerr << "d"<< deleteElements[i-1] << " ";
    }
      clusters.push_back(cluster);
  }
  
//   //calculate mean features
//   vector<Narf>  meanFeature;
//   vector<double>  meanPointCount;
//   meanFeature.resize(clusters.size());
//   meanPointCount.resize(clusters.size());
// 
//   if(clusters.size() == 0)
//     return;
//   int dsize = clusters[0][0]->feature.getDescriptorSize();  
//   for(size_t i = 0; i < clusters.size(); ++i){
//     vector<NarfWord* >& cluster = clusters[i];
//     float* mean = new float[dsize];
//     for(size_t k = 0; k < dsize; ++k)
//       mean[k] = 0;
// 
//     int count = 0;
//     double pointCount = 0.0;
//     for(size_t j = 0; j < cluster.size(); ++j){
//       float* descriptor = cluster[j]->feature.getDescriptor();
//       if(descriptor)
//         for(int k = 0; k < dsize; ++k){
//           mean[k] += descriptor[k];
//         }
//         count++;
//         pointCount += cluster[j]->pointCloud.size();
//     }
// 
//     for(size_t k = 0; k < dsize; ++k){
//       mean[k] = mean[k] / (double) count;
//     }
//     meanFeature[i].setDescriptor(mean);
//     meanFeature[i].getDescriptorSize() = dsize;
//     meanPointCount[i] = pointCount / (double) count;
//   }
// 
//   //select
//   for(size_t i = 0; i < clusters.size(); ++i){
//     vector<NarfWord* >& cluster = clusters[i];
//     if(cluster.size() == 1){
//       _dictionary.push_back(cluster[0]);
//     } else {
//       double minDist = 1e30;
//       NarfWord* representant = 0; 
//       for(size_t j = 0; j < cluster.size(); ++j){
//           double dist = meanFeature[i].getDescriptorDistance(cluster[j]->feature);
//           if(dist < minDist && cluster[j]->pointCloud.size() >= meanPointCount[i]){
//             minDist = dist;
//             representant = cluster[j];
//           }
//       }
//       if(representant == 0){
//         cerr << cluster.size() << " Error no representant for cluster found. THIS SHOULD NOT HAPPEN" << endl;
//         exit(1);
//       }
//       _dictionary.push_back(representant);
//     }
//   }

  cerr << "Found " << clusters.size() << " candidate clusters."<< endl;
  for(size_t i = 0; i < clusters.size(); ++i){
    //if(clusters[i].size() > 5)
    _dictionary.push_back(clusters[i][0]);
  }
  //vector<double>  histogram = model.getWordHistogram(_dictionary);
//    _dictionaryBackup = _dictionary;
//    _dictionary.clear();
//   for(size_t i = 0; i < _dictionaryBackup.size(); ++i){
//     //if(histogram[i+1] > 0.0002)
//       _dictionary.push_back(_dictionaryBackup[i]);
//   }

  cerr << "resulting dictionary size is " << _dictionary.size() << "." << endl;

//   for(size_t i = 1; i < clusters.size(); ++i){
//     _dictionary.push_back(clusters[i][0]);
//   }
}

void SPMDictionarySelector::clusterDictionaryWithkMeans(SPModel& model, int numOfClusters, int attempts){

  if(model._dictionary.size() == 0)
    return;

  vector<NarfWord* >& dictionary = model._dictionary;
  int elements = dictionary.size();
  int dimensions = dictionary[0]->feature.getDescriptorSize();

  //allocate memory
  Scalar* points = (Scalar*)malloc(elements * dimensions * sizeof(Scalar));

  for(int i = 0; i < elements; ++i){
    NarfWord*& word = dictionary[i];
    for(int j = 0; j < dimensions; ++j){
      // points: An array of size n*d where points[d*i + j] gives coordinate j of point i
     (points)[i * dimensions + j] = word->feature.getDescriptor()[j];
    }
  }
  
  Scalar *centers = (Scalar*)malloc(numOfClusters * dimensions * sizeof(Scalar));
  int *assignments = new int[elements];

  int verbosity = 3;
  AddKMeansLogging(&cout, verbosity == 3);
  std::cout << "clustering with k-means: " << endl;
  RunKMeansPlusPlus(elements, numOfClusters, dimensions, points, attempts, centers, assignments);

  std::map<int, int> representantId;
  _dictionary.clear();
  std::cout << "searching best descriptions for clusters" << endl;
  for(int k = 0; k < numOfClusters; ++k){
    double minDistance = 1e10;
    for(int i = 0; i < elements; ++i){
      NarfWord*& word = dictionary[i];
      double distance = 0.0;
      for(int j = 0; j < dimensions; ++j){
        distance += (word->feature.getDescriptor()[j] - centers[k * dimensions + j]) * (word->feature.getDescriptor()[j] - centers[k * dimensions + j]);
      }
      distance /= dimensions;
      
      if(distance < minDistance){
        representantId[k] = i;
        minDistance = distance;
      }
    }
    _dictionary.push_back(dictionary[representantId[k]]);
    //std::cout << _dictionary.size() << std::endl;
  }

  std::swap(dictionary, _dictionary);
  
  std::cout << "Transfering clusters to dictionary..." << endl;
  std::map<int, bool> clusterHasElement;
  for(int i = 0; i < elements; ++i){
    //int cluster = assignments[i];
    //std::cout << assignments[i] << " " << std::endl;
    model._instances[i].word = model._dictionary[assignments[i]];
    model._instances[i].fid = model._dictionary[assignments[i]]->id;
  }

  //Update wordMap
  model._wordMap.clear();
  for(size_t i = 0; i < model._dictionary.size(); ++i){
    model._wordMap[_dictionary[i]->id] = model._dictionary[i];
  }
  
  std::cout << "resulting dictionary size is " << model._dictionary.size() << "." << endl;
  
  delete[] points;
  delete[] assignments;
  //model.importDictionary(_dictionary);
}


void SPMDictionarySelector::fillModelPointCloudForDictionary(SPModel& model, SPModel::pclPointCloud& cloud, std::vector<NarfWord*>& dictionary, double& resolution)
{

  if(dictionary.size() == 0)
  {
    cerr << __PRETTY_FUNCTION__ << " ERROR: " << endl;
    cerr << PVAR(dictionary.size()) << " " << PVAR(cloud.size()) << " " << PVAR(resolution) << endl;
    return;
  }

  std::map<int, std::map<int, std::map<int, bool> > > pointMap;

  vector<NarfInstance>& instances = model._instances;
  for(unsigned int j = 0; j < instances.size(); j++){
    NarfInstance& instance = instances[j];
    NarfWord* wordForInstance = 0;

    //double minDistance = 1e30;
    double maxLikelihood = -1e30;
    //Get the dictionary word with the lowest descriptor distance
    double sigma = 0.000001; 
    for(size_t i = 0; i < dictionary.size(); ++i){
        NarfWord*& word = dictionary[i];
        
        double distance = 0.0;
        //if(!likelihoodForInstances[instance.fid][word->id])
        if(likelihoodForInstances[instance.fid].find(word->id) == likelihoodForInstances[instance.fid].end()){
          likelihoodForInstances[instance.fid][word->id] = LikelihoodModels::sphericalGaussian(word->pointCloud, instance.word->pointCloud, sigma);
        }
        distance = likelihoodForInstances[instance.fid][word->id];
        //distance = word->feature.getDescriptorDistance(instance.word->feature);
        if(distance > maxLikelihood){ //distance < minDistance ||
          wordForInstance = word;
          //minDistance = distance;
          maxLikelihood = distance;
        }
    }
    if(wordForInstance == 0){
      cerr << "No word for instance " << instance.fid << std::endl;
      cerr << "This should not happen!!" << endl;
      exit(0);
      break;
    }
    pcl::PointCloud<pcl::PointXYZ> tmpCloud;
    pcl::transformPointCloud(wordForInstance->pointCloud, tmpCloud, instance.pose);

    //Eigen::Vector3f p = instance.pose.translation();
    for(size_t i = 0; i < tmpCloud.size(); ++i)
    {
      Eigen::Vector3i index;
      model.pose2mapIndex(tmpCloud.points[i], index, resolution);
      if(!pointMap[index(0)][index(1)][index(2)]){
          cloud.push_back(tmpCloud.points[i]);
          pointMap[index(0)][index(1)][index(2)] = true;
      }
    }
  }
//   cerr << "resulting pointCloud has " << cloud.size() << " points." << std::endl;
}


void SPMDictionarySelector::optimizeInstancePoses(SPModel& model, pcl::PointCloud<pcl::PointXYZ>& cloud){
  IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputTarget(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));

  icp.setMaximumIterations(15);
  for(size_t i = 0; i < model._instances.size(); ++i){
    NarfInstance& instance = model._instances[i];
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::transformPointCloud(instance.word->pointCloud, tmp, instance.pose);
    icp.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (tmp));
    icp.align(tmp);
    instance.pose = Eigen::Affine3f(icp.getFinalTransformation()) * instance.pose;
  }
  
}
