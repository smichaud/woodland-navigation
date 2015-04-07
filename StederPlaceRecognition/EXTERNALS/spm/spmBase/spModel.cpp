#include "spModel.h"
#include "narfWord.h"
#include "narfInstance.h"
#include <omp.h>
#include "likelihoodModels.h"
#include "basics/timeutil.h"
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

SPModel::SPModel(){
  _maxWordId = 0;
}

SPModel::~SPModel(){
  clearInstances();
}

void SPModel::clear(){
  //clear instances
  std::vector<NarfInstance>   instances;
  std::swap(instances, _instances);
  //clear dictionary
  for(vector<NarfWord* >::iterator it = _dictionary.begin(); it != _dictionary.end(); ){
    delete (*it);
    it = _dictionary.erase(it);
  }
  _wordMap.clear();
  _maxWordId = 0;
}

void SPModel::clearInstances(){
  //clear instances
  std::vector<NarfInstance>   instances;
  std::swap(instances, _instances);
}

void SPModel::addInstanceAndWord(NarfInstance& instance)
{
  if(instance.fid == 0 || _wordMap[instance.fid] != 0){
    _maxWordId++;
    instance.fid = _maxWordId;
    instance.word->id = _maxWordId;
  }
  addInstance(instance);
  addWord(instance.word);
}

void SPModel::addInstance(NarfInstance& instance)
{
  _instances.push_back(instance);
}

void SPModel::addWord(NarfWord*& word)
{
  _dictionary.push_back(word);
  _wordMap[word->id] = word;
}

void SPModel::deleteInstance(NarfInstance& instance){
  instance.deleted = true;
  deleteMarkedInstances();
}

void SPModel::deleteMarkedInstances(){
  for(vector<NarfInstance>::iterator it = _instances.begin(); it != _instances.end(); ){
      if((*it).deleted){
        it = _instances.erase(it);
      } else {
        it++;
      }
  }
}

void SPModel::deleteWord(NarfWord*& word){
  //First check if word is not used any more
  for(vector<NarfInstance>::iterator it = _instances.begin(); it != _instances.end(); ++it){
    if((*it).fid == word->id){
      cout << "Warning: could not delete word with id " << word->id << ". Word is still linked to instance " << (*it).fid << "." << endl;
      return;
    }
  }
  word->deleted = true;
  deleteMarkedWords();
  delete word;
}

void SPModel::deleteMarkedWords(){
  for(vector<NarfWord* >::iterator it = _dictionary.begin(); it != _dictionary.end(); ){
      if((*it)->deleted){
        delete (*it);
        it = _dictionary.erase(it);
      } else {
        it++;
      }
  }
}

SPModel::pclPointCloud SPModel::getModelPointCloud()
{
//   pclPointCloud cloud;
//   for(unsigned int j = 0; j < _instances.size(); j++){
//     NarfInstance& instance = _instances[j];
//     pclPointCloud tmpCloud;
//     pcl::transformPointCloud(instance.word->pointCloud, tmpCloud, instance.pose);
//     std::copy(tmpCloud.points.begin(), tmpCloud.points.end(), back_inserter(cloud.points));
//   }
  double resolution = 0.01;
  return getModelPointCloud(resolution);
}

SPModel::pclPointCloud SPModel::getModelPointCloud(double& resolution)
{
  pclPointCloud cloud;
  std::map<int, std::map<int, std::map<int, bool> > > pointMap;

  for(unsigned int j = 0; j < _instances.size(); j++){
    NarfInstance& instance = _instances[j];
    pclPointCloud tmpCloud;
    pcl::transformPointCloud(instance.word->pointCloud, tmpCloud, instance.pose);

    //Eigen::Vector3f p = instance.pose.translation();
    for(size_t i = 0; i < tmpCloud.size(); ++i)
    {
      Eigen::Vector3i index;
      pose2mapIndex(tmpCloud.points[i], index, resolution);
      if(!pointMap[index(0)][index(1)][index(2)]){
          cloud.push_back(tmpCloud.points[i]);
          pointMap[index(0)][index(1)][index(2)] = true;
      }
    }
  }
  cerr << "resulting pointCloud has " << cloud.size() << " points." << std::endl;
  return cloud;
}

SPModel::pclPointCloud SPModel::getModelPointCloudForDictionary(std::vector<NarfWord*>& dictionary, double& resolution)
{
  pclPointCloud cloud;
  fillModelPointCloudForDictionary(cloud, dictionary, resolution);
  return cloud;
}

void SPModel::fillModelPointCloudForDictionary(SPModel::pclPointCloud& cloud, std::vector<NarfWord*>& dictionary, double& resolution)
{

  if(dictionary.size() == 0)
  {
    cerr << __PRETTY_FUNCTION__ << " ERROR: " << endl;
    cerr << PVAR(dictionary.size()) << " " << PVAR(cloud.size()) << " " << PVAR(resolution) << endl;
    return;
  }

  std::map<int, std::map<int, std::map<int, bool> > > pointMap;
  InstanceWordAssignmentStrategie strategie = DescriptorDistance;
  std::map<int, int> instance2Word = getInstance2WordMap(_instances, dictionary, strategie);

  for(unsigned int j = 0; j < _instances.size(); j++){
    NarfInstance& instance = _instances[j];
    int id = instance2Word[instance.fid];
    NarfWord* wordForInstance = dictionary[id];

    pclPointCloud tmpCloud;
    pcl::transformPointCloud(wordForInstance->pointCloud, tmpCloud, instance.pose);

    //Eigen::Vector3f p = instance.pose.translation();
    for(size_t i = 0; i < tmpCloud.size(); ++i)
    {
      Eigen::Vector3i index;
      pose2mapIndex(tmpCloud.points[i], index, resolution);
      if(!pointMap[index(0)][index(1)][index(2)]){
          cloud.push_back(tmpCloud.points[i]);
          pointMap[index(0)][index(1)][index(2)] = true;
      }
    }
  }
//   cerr << "resulting pointCloud has " << cloud.size() << " points." << std::endl;
}

SPModel::pclPointCloud SPModel::getInstancePointCloud()
{
  pclPointCloud cloud;
  for(unsigned int j = 0; j < _instances.size(); j++){
    NarfInstance& instance = _instances[j];
    Eigen::Vector3f p = instance.pose.translation();
    cloud.push_back(pcl::PointXYZ(p(0), p(1), p(2)));
  }
  return cloud;
}

void SPModel::pose2mapIndex(pcl::PointXYZ& point, Eigen::Vector3i& index, double& resolution)
{
  index(0) = lrint(point.x / resolution);
  index(1) = lrint(point.y / resolution);
  index(2) = lrint(point.z / resolution);
}




  void SPModel::instancesToPointCloud(vector<NarfInstance>& scanInstances, pclPointCloud& cloud){
    pcl::PointXYZ point;
    for(unsigned int i = 0; i < scanInstances.size(); i++){
      NarfInstance& elementA = scanInstances[i];
      Eigen::Vector3f p = elementA.pose.translation();
      pcl::PointXYZ point(p(0), p(1), p(2));
      cloud.push_back(point);
    }
  }  

std::map<int, int> SPModel::getInstance2WordMap(vector<NarfInstance>& scanInstances, vector<NarfWord* >& dictionary, InstanceWordAssignmentStrategie strategie = DescriptorDistance){
  std::map<int, int> wordForInstance;
  for(unsigned int j = 0; j < scanInstances.size(); j++){
    NarfInstance& instance = scanInstances[j];

    if(strategie == DescriptorDistance)
      wordForInstance[instance.fid] = getNarfWordIdForInstance(instance, dictionary);

    if(strategie == MaximumLikelihood)
      wordForInstance[instance.fid] = getNarfWordIdForInstanceMaximumLikelihood(instance, dictionary);
  }
  return wordForInstance;
}

int SPModel::getNarfWordIdForInstance(NarfInstance& instance, vector<NarfWord* >& dictionary){
  unsigned int id = 0;
  double minDistance = 1e30;

  //std::cout << "Dictionary size "<<dictionary.size()<<"\n";

  //Get the _dictionary word with the lowest descriptor distance
  for(size_t i = 0; i < dictionary.size(); ++i){
      NarfWord*& word = dictionary[i];
    if(word == 0){
      cerr << "Warning corrupt dictionary!" << endl;
      continue;
    }
    if(instance.word == 0){
      cerr << "Warning corrupt scene instances!" << endl;
      continue;
    }
    if(word->feature.getDescriptorSize() != instance.word->feature.getDescriptorSize()){
      cerr << "Warning descriptor sizes do not match " << word->feature.getDescriptorSize() << " " << instance.word->feature.getDescriptorSize() << endl;
    }
    double distance = word->feature.getDescriptorDistance(instance.word->feature);
      if(distance < minDistance){
        minDistance = distance;
        id = i;
      }
  }
  return id;
}

int SPModel::getNarfWordIdForInstanceMaximumLikelihood(NarfInstance& instance, vector<NarfWord* >& dictionary){
  unsigned int id = 0;
  double maxLikelihood = -1e30;
  double sigma = 0.00001; 
  std::map<unsigned int, std::map<unsigned int, double> > likelihoodForInstances;

  for(size_t i = 0; i < dictionary.size(); ++i){
      NarfWord*& word = dictionary[i];
      double distance = LikelihoodModels::sphericalGaussian(word->pointCloud, instance.word->pointCloud, sigma);
      if(distance > maxLikelihood){
        maxLikelihood = distance;
        id = i;
      }
  }
  return id;
}

   void SPModel::writeBinary(std::ostream &file) const {
      //Features
      unsigned int size = _dictionary.size();
      file.write((char*)&size, sizeof(size));
      //std::cerr << PVAR(size) << std::endl;      
      for(unsigned int i = 0; i < size; i++){
        _dictionary[i]->writeBinary(file);
      }

      size = _instances.size();
      file.write((char*)&size, sizeof(size));
      //std::cerr << PVAR(size) << std::endl;
      for(unsigned int i = 0; i < size; i++){
         _instances[i].writeBinary(file);
      }
   }

   void SPModel::readBinary(std::istream &file){
      double time = Ais3dTools::get_time();
      unsigned int size = 0;
      float support_size = 0.0f;
      file.read((char*)&size, sizeof(size));
      //std::cerr << PVAR(size) << std::endl;
      for(unsigned int i = 0; i < size; i++){
          NarfWord* feature = new NarfWord;
          feature->readBinary(file);
         _dictionary.push_back(feature);
         if(i==0)
           support_size = feature->feature.getSurfacePatchWorldSize();
      }
      file.read((char*)&size, sizeof(size));
      //std::cerr << PVAR(size) << std::endl;
      for(unsigned int i = 0; i < size; i++){
          NarfInstance element;
          element.readBinary(file);
         _instances.push_back(element);
      }
      cerr << "[spm loading took: " << Ais3dTools::get_time() - time << "s | dictionary: "<< _dictionary.size() <<" | instances: " << _instances.size() << " | support_size: " << support_size << "]" << endl;
      linkWordsAndInstances();
   }

void SPModel::linkWordsAndInstances(){
  for(unsigned int j = 0; j < _dictionary.size(); j++)
    _wordMap[_dictionary[j]->id ] = _dictionary[j];

  for(unsigned int i = 0; i < _instances.size(); i++){
    _instances[i].word = _wordMap[_instances[i].fid];
    if(_instances[i].word == 0){
      cerr << "No feature found for element " << i << " with id " << _instances[i].fid << std::endl;
    }
    }
  }

void SPModel::stats(){
  unsigned int pointCount = 0;
  if(_dictionary.size() > 0)
  {
    pcl::Narf& feature = _dictionary[0]->feature;
    double featureWorldSize = feature.getSurfacePatchWorldSize();
    unsigned int featurePixelSize = feature.getDescriptorSize();
    cerr << "feature size: " << featureWorldSize << " descriptor size: " << featurePixelSize << std::endl;
    for(unsigned int i = 0; i < _dictionary.size(); i++){ 
        pointCount += _dictionary[i]->pointCloud.size();
    }
  }
  cout << "Dictionary with "<< _dictionary.size() << " words and " << pointCount << " points." << std::endl;
  cout << "Word instances in model " << _instances.size() << " poses."  << std::endl;
  cout << "theoreticall minimum size: " << _instances.size() * 7 * 4 + pointCount * 12 << " bytes" << endl;
}

void SPModel::importDictionary(std::vector<NarfWord*>& dictionary)
{
  std::cout << _instances.size()<<" instances.\n";
  for(unsigned int j = 0; j < _instances.size(); j++){
    int id = getNarfWordIdForInstance(_instances[j], dictionary);
    //int id = getNarfWordIdForInstanceMaximumLikelihood(_instances[j], dictionary);
    //std::cout << j<<","<<id<<"\n";
    NarfWord* wordForInstance = dictionary[id];
    _instances[j].word = wordForInstance;
    _instances[j].fid = wordForInstance->id;

    if(wordForInstance == 0){
      cerr << "No word for instance " << _instances[j].fid << std::endl;
      cerr << "This should not happen!!" << endl;
      exit(0);
      break;
    }
  }
  _dictionary = dictionary;

  //Update wordMap
  _wordMap.clear();
  for(size_t i = 0; i < _dictionary.size(); ++i){
      _wordMap[_dictionary[i]->id] = _dictionary[i];
  }
}

void SPModel::sortDictionary(){
  countWordOccurances();
  sort(_dictionary.begin(), _dictionary.end(), NarfWord::NarfWordCmp_);
}

void SPModel::countWordOccurances(){
  for(unsigned int j = 0; j < _instances.size(); j++){   
      _instances[j].word->count = 0;
  }
  for(unsigned int j = 0; j < _instances.size(); j++){
    _instances[j].word->count++;
  }
}

void SPModel::deleteUnusedWords(){
  countWordOccurances();
  for(unsigned int j = 1; j < _dictionary.size(); j++){
    if(_dictionary[j]->count == 0){
      _dictionary[j]->deleted = true;
    }
  }
  deleteMarkedWords();
}


void SPModel::sparseInstances(double support_size){
  //double support_size = support_size_dictionary[0]->feature.; //.getDescriptorSize();
  //Sparse instances
  SPModel::pclPointCloud instance_cloud;
  instancesToPointCloud(_instances, instance_cloud);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (instance_cloud.makeShared ());

  std::vector<bool> removeInstance(_instances.size());
  for(size_t i = 0; i < _instances.size(); ++i)
    removeInstance[i] = false;

  for(size_t i = 0; i < instance_cloud.size(); ++i){
    if(removeInstance[i])
      continue;
    vector<int> k_indices;
    vector<float> k_distances;
    kdtree.radiusSearch (instance_cloud, i, (support_size * 0.3), k_indices, k_distances);
    for(size_t j = 0; j < k_indices.size(); ++j){
      size_t id = k_indices[j];
      //std::cerr << PVAR(i) << " " << PVAR(id) << " " << k_distances[j] << " " << support_size << std::endl;
      if(!removeInstance[id] && i != id){        
        double desDistance = _instances[i].word->feature.getDescriptorDistance(_instances[id].word->feature);
        if(desDistance < 0.05){
          if(_instances[i].word->pointCloud.size() > _instances[id].word->pointCloud.size()){
             removeInstance[id] = true;
          } else {
             removeInstance[i] = true;
             continue;
          }
        }
      }
    }
  }

  for(size_t i = 0; i < _instances.size(); ++i){
    if(removeInstance[i])
      _instances[i].deleted = true;
  }
  deleteMarkedInstances();
}


std::vector<double> SPModel::getWordHistogram(std::vector<NarfWord* >& dictionary){
  std::vector<double> histogram;
  for(size_t i = 0; i < dictionary.size(); ++i){
    histogram.push_back(0);
  }

  histogram.push_back(0); 
  int count = 0;
  for(size_t i = 0; i < _instances.size(); i++){
    NarfInstance& instance = _instances[i];
    int id = (int) getNarfWordIdForInstance(instance, dictionary);
    histogram[id] += 1.0;
    count++;
  }
  //Normalizing
  for(unsigned int i = 0; i < histogram.size(); i++){
    histogram[i] =  histogram[i] / (double) count;
  }
  histogram[0] = (double) count;
  return histogram;
}

