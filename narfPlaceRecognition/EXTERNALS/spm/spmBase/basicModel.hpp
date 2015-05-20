#include <ios>

template<typename DescriptionType>
BasicModelT<DescriptionType>::BasicModelT() : _wordZOffset(0.0){
  _maxWordId = 0;
}

template<typename DescriptionType>
BasicModelT<DescriptionType>::~BasicModelT(){
  clearInstances();
}

template<typename DescriptionType>
void BasicModelT<DescriptionType>::clear(){
  //clear instances
  typename std::vector<BasicInstanceT<DescriptionType> >   instances;
  std::swap(instances, _instances);
  //clear dictionary
  for(typename std::vector<BasicWordT<DescriptionType>* >::iterator it = _dictionary.begin(); it != _dictionary.end(); ){
    delete (*it);
    it = _dictionary.erase(it);
  }
  _wordMap.clear();
  _maxWordId = 0;
}

template<typename DescriptionType>
void BasicModelT<DescriptionType>::clearInstances(){
  //clear instances
  std::vector<BasicInstanceT<DescriptionType> >   instances;
  std::swap(instances, _instances);
}

template<typename DescriptionType>
void BasicModelT<DescriptionType>::addInstanceAndWord(BasicInstanceT<DescriptionType> & instance)
{
  if(instance.fid == 0 || _wordMap[instance.fid] != 0){
    _maxWordId++;
    instance.fid = _maxWordId;
    instance.word->id = _maxWordId;
  }
  addInstance(instance);
  addWord(instance.word);
}

template<typename DescriptionType>
void BasicModelT<DescriptionType>::addInstance(BasicInstanceT<DescriptionType> & instance)
{
  _instances.push_back(instance);
}

template<typename DescriptionType>
void BasicModelT<DescriptionType>::addWord(BasicWordT<DescriptionType>*& word)
{
  _dictionary.push_back(word);
  _wordMap[word->id] = word;
}

// template<typename DescriptionType>
// void BasicModelT<DescriptionType>::deleteInstance(BasicInstanceT<DescriptionType> & instance){
//   instance.deleted = true;
//   deleteMarkedInstances();
// }

// template<typename DescriptionType>
// void BasicModelT<DescriptionType>::deleteMarkedInstances(){
//   for(vector<BasicInstanceT<DescriptionType> >::iterator it = _instances.begin(); it != _instances.end(); ){
//       if((*it).deleted){
//         it = _instances.erase(it);
//       } else {
//         it++;
//       }
//   }
// }

// template<typename DescriptionType>
// void BasicModelT<DescriptionType>::deleteWord(BasicWordT<DescriptionType>*& word){
//   //First check if word is not used any more
//   for(vector<BasicInstanceT<DescriptionType> >::iterator it = _instances.begin(); it != _instances.end(); ++it){
//     if((*it).fid == word->id){
//       cout << "Warning: could not delete word with id " << word->id << ". Word is still linked to instance " << (*it).fid << "." << endl;
//       return;
//     }
//   }
//   word->deleted = true;
//   deleteMarkedWords();
//   delete word;
// }
// 
// template<typename DescriptionType>
// void BasicModelT<DescriptionType>::deleteMarkedWords(){
//   for(vector<BasicWordT<DescriptionType>* >::iterator it = _dictionary.begin(); it != _dictionary.end(); ){
//       if((*it)->deleted){
//         delete (*it);
//         it = _dictionary.erase(it);
//       } else {
//         it++;
//       }
//   }
// }
// /*
// SPModel::pclPointCloud SPModel::getModelPointCloud()
// {
// //   pclPointCloud cloud;
// //   for(unsigned int j = 0; j < _instances.size(); j++){
// //     BasicInstanceT<DescriptionType> & instance = _instances[j];
// //     pclPointCloud tmpCloud;
// //     pcl::transformPointCloud(instance.word->pointCloud, tmpCloud, instance.pose);
// //     std::copy(tmpCloud.points.begin(), tmpCloud.points.end(), back_inserter(cloud.points));
// //   }
//   double resolution = 0.01;
//   return getModelPointCloud(resolution);
// }
// 
template<typename DescriptionType>
void BasicModelT<DescriptionType>::getModelPointCloud(pclPointCloud& cloud, double resolution)
{
  double sTime = get_time();
  std::map<int, std::map<int, std::map<int, bool> > > pointMap;

  for(unsigned int j = 0; j < _instances.size(); j++){
    BasicInstanceT<DescriptionType> & instance = _instances[j];
//     pclPointCloud tmpCloud;
//     pcl::transformPointCloud(instance.word->data, tmpCloud, instance.pose);

    for(size_t i = 0; i < instance.word->data.size(); ++i)
    {
      PointType p = instance.word->data[i];
      p.getVector3fMap() = instance.pose * p.getVector3fMap();
      Eigen::Vector3i index;
      pose2mapIndex(p, index, resolution);
      if(!pointMap[index(0)][index(1)][index(2)]){
          cloud.push_back(p);
          pointMap[index(0)][index(1)][index(2)] = true;
      }
    }
  }
  std::cerr << "resulting pointCloud has " << cloud.size() << " points. Creation took " << (get_time() - sTime) << "s." << std::endl;
}
// 
// SPModel::pclPointCloud SPModel::getModelPointCloudForDictionary(std::vector<BasicWordT<DescriptionType>*>& dictionary, double& resolution)
// {
//   pclPointCloud cloud;
//   fillModelPointCloudForDictionary(cloud, dictionary, resolution);
//   return cloud;
// }
// 
// template<typename DescriptionType>
// BasicModelT<DescriptionType>::fillModelPointCloudForDictionary(SPModel::pclPointCloud& cloud, std::vector<BasicWordT<DescriptionType>*>& dictionary, double& resolution)
// {
// 
//   if(dictionary.size() == 0)
//   {
//     cerr << __PRETTY_FUNCTION__ << " ERROR: " << endl;
//     cerr << PVAR(dictionary.size()) << " " << PVAR(cloud.size()) << " " << PVAR(resolution) << endl;
//     return;
//   }
// 
//   std::map<int, std::map<int, std::map<int, bool> > > pointMap;
//   InstanceWordAssignmentStrategie strategie = DescriptorDistance;
//   std::map<int, int> instance2Word = getInstance2WordMap(_instances, dictionary, strategie);
// 
//   for(unsigned int j = 0; j < _instances.size(); j++){
//     BasicInstanceT<DescriptionType> & instance = _instances[j];
//     int id = instance2Word[instance.fid];
//     BasicWordT<DescriptionType>* wordForInstance = dictionary[id];
// 
//     pclPointCloud tmpCloud;
//     pcl::transformPointCloud(wordForInstance->pointCloud, tmpCloud, instance.pose);
// 
//     Eigen::Vector3f p = instance.pose.translation();
//     for(size_t i = 0; i < tmpCloud.size(); ++i)
//     {
//       Eigen::Vector3i index;
//       pose2mapIndex(tmpCloud.points[i], index, resolution);
//       if(!pointMap[index(0)][index(1)][index(2)]){
//           cloud.push_back(tmpCloud.points[i]);
//           pointMap[index(0)][index(1)][index(2)] = true;
//       }
//     }
//   }
// //   cerr << "resulting pointCloud has " << cloud.size() << " points." << std::endl;
// }
// 
// SPModel::pclPointCloud SPModel::getInstancePointCloud()
// {
//   pclPointCloud cloud;
//   for(unsigned int j = 0; j < _instances.size(); j++){
//     BasicInstanceT<DescriptionType> & instance = _instances[j];
//     Eigen::Vector3f p = instance.pose.translation();
//     cloud.push_back(pcl::PointXYZ(p(0), p(1), p(2)));
//   }
//   return cloud;
// }*/

template<typename DescriptionType>
void BasicModelT<DescriptionType>::pose2mapIndex(PointType& point, Eigen::Vector3i& index, double& resolution)
{
  index(0) = lrint(point.x / resolution);
  index(1) = lrint(point.y / resolution);
  index(2) = lrint(point.z / resolution);
}



/*
  template<typename DescriptionType>
BasicModelT<DescriptionType>::instancesToPointCloud(vector<BasicInstanceT<DescriptionType> >& scanInstances, pclPointCloud& cloud){
    pcl::PointXYZ point;
    for(unsigned int i = 0; i < scanInstances.size(); i++){
      BasicInstanceT<DescriptionType> & elementA = scanInstances[i];
      Eigen::Vector3f p = elementA.pose.translation();
      pcl::PointXYZ point(p(0), p(1), p(2));
      cloud.push_back(point);
    }
  }  

std::map<int, int> SPModel::getInstance2WordMap(vector<BasicInstanceT<DescriptionType> >& scanInstances, vector<BasicWordT<DescriptionType>* >& dictionary, InstanceWordAssignmentStrategie strategie = DescriptorDistance){
  std::map<int, int> wordForInstance;
  for(unsigned int j = 0; j < scanInstances.size(); j++){
    BasicInstanceT<DescriptionType> & instance = scanInstances[j];

    if(strategie == DescriptorDistance)
      wordForInstance[instance.fid] = getBasicWordT<DescriptionType>IdForInstance(instance, dictionary);

    if(strategie == MaximumLikelihood)
      wordForInstance[instance.fid] = getBasicWordT<DescriptionType>IdForInstanceMaximumLikelihood(instance, dictionary);
  }
  return wordForInstance;
}

int SPModel::getBasicWordT<DescriptionType>IdForInstance(BasicInstanceT<DescriptionType> & instance, vector<BasicWordT<DescriptionType>* >& dictionary){
  unsigned int id = 0;
  double minDistance = 1e30;

  Get the _dictionary word with the lowest descriptor distance
  for(size_t i = 0; i < dictionary.size(); ++i){
      BasicWordT<DescriptionType>*& word = dictionary[i];
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

int SPModel::getBasicWordT<DescriptionType>IdForInstanceMaximumLikelihood(BasicInstanceT<DescriptionType> & instance, vector<BasicWordT<DescriptionType>* >& dictionary){
  unsigned int id = 0;
  double maxLikelihood = -1e30;
  double sigma = 0.00001; 
  std::map<unsigned int, std::map<unsigned int, double> > likelihoodForInstances;

  for(size_t i = 0; i < dictionary.size(); ++i){
      BasicWordT<DescriptionType>*& word = dictionary[i];
      double distance = LikelihoodModels::sphericalGaussian(word->pointCloud, instance.word->pointCloud, sigma);
      if(distance > maxLikelihood){
        maxLikelihood = distance;
        id = i;
      }
  }
  return id;
}*/

template<typename DescriptionType>
void BasicModelT<DescriptionType>::writeBinary(std::ostream &file) const {
      //Features
      unsigned int size = _dictionary.size();
      file.write((char*)&size, sizeof(size));
      for(unsigned int i = 0; i < size; i++){
	_dictionary[i]->writeBinary(file);
      }
      size = _instances.size();
      file.write((char*)&size, sizeof(size));
      for(unsigned int i = 0; i < size; i++){
        _instances[i].writeBinary(file);
      }
   }

template<typename DescriptionType>
void BasicModelT<DescriptionType>::readBinary(std::istream &file){
      double time = Ais3dTools::get_time();
      unsigned int size = 0;
      file.read((char*)&size, sizeof(size));
      for(unsigned int i = 0; i < size; i++){
        BasicWordT<DescriptionType>* feature = new BasicWordT<DescriptionType>;
        feature->readBinary(file);
        _dictionary.push_back(feature);
      }
      file.read((char*)&size, sizeof(size));
      for(unsigned int i = 0; i < size; i++){
        BasicInstanceT<DescriptionType>  element;
        element.readBinary(file);
        _instances.push_back(element);
      }
      std::cerr << "[spm loading took: " << Ais3dTools::get_time() - time << "s | dictionary: "<< _dictionary.size() <<" | instances: " << _instances.size() << "]" << std::endl;
      linkWordsAndInstances();
   }

template<typename DescriptionType>
void BasicModelT<DescriptionType>::exportWordsAscii(std::ostream &file) const {
    //Features
    unsigned int size = _dictionary.size();
    for(unsigned int i = 0; i < size; i++){
       {
	for(unsigned int j = 0; j < _dictionary[i]->data.points.size(); ++j){
	  float value = 0.0;
	  if(pcl::isFinite(_dictionary[i]->data.points[j]))
	    value = _dictionary[i]->data.points[j].z;
	  file << value + _wordZOffset<< " ";
	}
	file << std::endl;
       }
    }
}

template<typename DescriptionType>
void BasicModelT<DescriptionType>::exportDefinedBitMaskAscii(std::ostream &file) const {
    //Features
    unsigned int size = _dictionary.size();
    for(unsigned int i = 0; i < size; i++){
       {
	for(unsigned int j = 0; j < _dictionary[i]->data.points.size(); ++j){
	  int value = 1.0;
	  if(!pcl::isFinite(_dictionary[i]->data.points[j]))
	    value = 0.0;
	  file << value << " ";
	}
	file << std::endl;
       }
    }
}

template<typename DescriptionType>
void BasicModelT<DescriptionType>::evaluateBitMaskAscii(std::istream &dictFile, std::istream &gammeFile, int wordWidth = 10)
{  
  std::vector<Eigen::MatrixXf > dictionary;
    while(dictFile.good()){
      Eigen::MatrixXf word(wordWidth, wordWidth);
      for(int j = 0; j < wordWidth; ++j)
	for(int k = 0; k < wordWidth; ++k){
	  double value;
	  dictFile >> value;
	  word(k,j) = value;
	}
      if(!dictFile.eof() && dictFile.good())
	dictionary.push_back(word);
    }
    std::cerr << "parsed " << dictionary.size() << " dictionary entries " << std::endl;

    float epsilon = 1e-8;
    int numLinearFactors = 0;
    std::vector<std::vector<int> > gamma_index;
    std::vector<std::vector<float> > gamma_value;
    while(gammeFile.good()){
      std::vector<int> 	indices;
      std::vector<float> linearFactor;
      for(size_t j = 0; j < dictionary.size(); ++j){
	  float cell;
	  gammeFile >> cell;
	  if(fabs(cell) > epsilon){
	    linearFactor.push_back(cell);
	    indices.push_back(j);
	    numLinearFactors++;
	  }
	}
      if(gammeFile.good()){
	gamma_value.push_back(linearFactor);
	gamma_index.push_back(indices);
      }
    }
    std::cerr << "parsed " << gamma_value.size() << " linear combination entries " << std::endl;

    float meanError = 0.0f;
    int count = 0;
    int bits = 0;
    for(int i = 0; i < static_cast<int>(instances().size()); i++){ //static_cast<int>(instances().size())
      DescriptionType& realDescriptor = instances()[i].word->data;
      std::vector<float>& linearFactors = gamma_value[i];
      std::vector<int>&   indices = gamma_index[i];
      
      Eigen::MatrixXf ksvdDescriptor(wordWidth,wordWidth);
      ksvdDescriptor.setZero();
      for(int j = 0; j < static_cast<int>(linearFactors.size()); j++){
	  ksvdDescriptor += linearFactors[j] * dictionary[indices[j]];
      }
     
      float error = 0.0f;
      float border = 0.5f;
      int pixelCount = 0;
      for(int y = 0; y < wordWidth; y++)
	for(int x = 0; x < wordWidth; x++){
	  if(ksvdDescriptor(x,y) < border && pcl::isFinite(realDescriptor.points[y * wordWidth + x])){
 	    error += 1.0f;
// 	    std::cerr << ksvdDescriptor(x,y) << " " << (int) pcl::isFinite(realDescriptor.points[y * wordWidth + x]) << std::endl;
	  }

	  if(ksvdDescriptor(x,y) >= border && !pcl::isFinite(realDescriptor.points[y * wordWidth + x])){
	    error += 1.0f;
// 	    std::cerr << ksvdDescriptor(x,y) << " " << (int) pcl::isFinite(realDescriptor.points[y * wordWidth + x]) << std::endl;
	  }
	  if(!pcl::isFinite(realDescriptor.points[y * wordWidth + x]))
	    bits++;
	  count++;
	}
      meanError += error;
      if(error > 0)
	std::cerr << "instance " << i << " differences " << (error) << " #l " << linearFactors.size() << std::endl;
    }
//     std::cerr << "current representation uses the following number of floats: " << std::endl;
//     std::cerr << dictionary.size() * wordWidth * wordWidth << " dictionary " << std::endl;
//     std::cerr << numLinearFactors << " linear factors " << std::endl;
//     std::cerr << instances().size() * 12 << " for word coordinates " << std::endl;
//     std::cerr << (dictionary.size() * wordWidth * wordWidth) + numLinearFactors + (instances().size() * 12) << " in total " << std::endl;
    std::cerr << std::fixed << "number of false bits:" << (meanError) << " " << count << " " << (float) meanError / (float) count << " number of raw bits " << bits << std::endl;    
}

template<typename DescriptionType>
void BasicModelT<DescriptionType>::importDictionaryAscii(std::istream &dictFile, std::istream &gammeFile, int wordWidth = 10)
{  
  //read in new dictionary
  std::vector<Eigen::MatrixXf > dictionary;
    while(dictFile.good()){
      Eigen::MatrixXf word(wordWidth, wordWidth);
      for(int j = 0; j < wordWidth; ++j)
	for(int k = 0; k < wordWidth; ++k){
	  double value;
	  dictFile >> value;
	  word(k,j) = value;
	}
	
//       std::cerr << dictionary.size() << " " << word.size() << " word size " << (int) dictFile.eof() << " " << (int) dictFile.fail() << " " << (int) dictFile.bad() << " " << (int) dictFile.good() << std::endl;
      if(!dictFile.eof() && dictFile.good())
	dictionary.push_back(word);
    }
    std::cerr << "parsed " << dictionary.size() << " dictionary entries " << std::endl;

    float epsilon = 1e-8;
    std::vector<std::vector<int> > gamma_index;
    std::vector<std::vector<float> > gamma_value;
    while(gammeFile.good()){
      std::vector<int> 	indices;
      std::vector<float> linearFactor;
      for(size_t j = 0; j < dictionary.size(); ++j){
	  float cell;
	  gammeFile >> cell;
	  if(fabs(cell) > epsilon){
	    linearFactor.push_back(cell);
	    indices.push_back(j);
	  }
	}
      if(gammeFile.good()){
	gamma_value.push_back(linearFactor);
	gamma_index.push_back(indices);
      }
    }
    std::cerr << "parsed " << gamma_value.size() << " linear combination entries " << std::endl;

    std::cerr << "transfer dictionary and linear factors to model dictionary \t";

    for(int i = 0; i < static_cast<int>(instances().size()); i++){
      DescriptionType& realDescriptor = instances()[i].word->data;
      std::vector<float>& linearFactors = gamma_value[i];
      std::vector<int>&   indices = gamma_index[i];
      
      Eigen::MatrixXf ksvdDescriptor(wordWidth,wordWidth);
      ksvdDescriptor.setZero();
      for(int j = 0; j < static_cast<int>(linearFactors.size()); j++){
	  ksvdDescriptor += linearFactors[j] * dictionary[indices[j]];
      }
     
      for(int y = 0; y < wordWidth; y++)
	for(int x = 0; x < wordWidth; x++){
 	  ksvdDescriptor(x,y) -= _wordZOffset;
	  if(!pcl::isFinite(realDescriptor.points[y * wordWidth + x]) ){ //HACK: to get rid of max range points
	    realDescriptor.points[y * wordWidth + x].z = std::numeric_limits<float>::quiet_NaN();
	  } else {
	    realDescriptor.points[y * wordWidth + x].z = ksvdDescriptor(x,y);
	  }
	}
    }
    std::cerr << " done " << std::endl;    
}

template<typename DescriptionType>
void BasicModelT<DescriptionType>::evaluateDictionaryAscii(std::istream &dictFile, std::istream &gammeFile, int wordWidth = 10)
{  
  std::vector<Eigen::MatrixXf > dictionary;
    while(dictFile.good()){
      Eigen::MatrixXf word(wordWidth, wordWidth);
      for(int j = 0; j < wordWidth; ++j)
	for(int k = 0; k < wordWidth; ++k){
	  double value;
	  dictFile >> value;
	  word(k,j) = value;
	}
      if(!dictFile.eof() && dictFile.good())
	dictionary.push_back(word);
    }
    std::cerr << "parsed " << dictionary.size() << " dictionary entries " << std::endl;

    float epsilon = 1e-8;
    int numLinearFactors = 0;
    std::vector<std::vector<int> > gamma_index;
    std::vector<std::vector<float> > gamma_value;
    while(gammeFile.good()){
      std::vector<int> 	indices;
      std::vector<float> linearFactor;
      for(size_t j = 0; j < dictionary.size(); ++j){
	  float cell;
	  gammeFile >> cell;
	  if(fabs(cell) > epsilon){
	    linearFactor.push_back(cell);
	    indices.push_back(j);
	    numLinearFactors++;
	  }
	}
      if(gammeFile.good()){
	gamma_value.push_back(linearFactor);
	gamma_index.push_back(indices);
      }
    }
    std::cerr << "parsed " << gamma_value.size() << " linear combination entries " << std::endl;

    float meanError = 0.0f;
    int count = 0;
    for(int i = 0; i < static_cast<int>(instances().size()); i++){ //static_cast<int>(instances().size())
      DescriptionType& realDescriptor = instances()[i].word->data;
      std::vector<float>& linearFactors = gamma_value[i];
      std::vector<int>&   indices = gamma_index[i];
      
      Eigen::MatrixXf ksvdDescriptor(wordWidth,wordWidth);
      ksvdDescriptor.setZero();
      for(int j = 0; j < static_cast<int>(linearFactors.size()); j++){
	  ksvdDescriptor += linearFactors[j] * dictionary[indices[j]];
      }
     
      float error = 0.0f;
      int pixelCount = 0;
      for(int y = 0; y < wordWidth; y++)
	for(int x = 0; x < wordWidth; x++){
// 	  if(ksvdDescriptor(x,y) != _wordZOffset
 	  ksvdDescriptor(x,y) -= _wordZOffset;
	  if(ksvdDescriptor(x,y) < -90.0f && !pcl::isFinite(realDescriptor.points[y * wordWidth + x]))
	    continue;
	  if(pcl::isFinite(realDescriptor.points[y * wordWidth + x])){
	    error += (realDescriptor.points[y * wordWidth + x].z - (ksvdDescriptor(x,y))) * (realDescriptor.points[y * wordWidth + x].z - (ksvdDescriptor(x,y)));
	    //std::cerr << std::fixed << pixelCount << " original: " << realDescriptor.points[y * wordWidth + x].z << " reconstructed: " << (ksvdDescriptor(x,y)) << " acc_error: " << error << std::endl;
	    pixelCount++;
	  }
	}
      error /= (float) pixelCount;
      error = sqrt(error);
      count++;
      meanError += error;
//       if(error > 0.001)
	//std::cerr << "instance " << i << " rmse " << (error) << " #l " << linearFactors.size() << std::endl;
    }
    meanError = meanError / (float) count;
    std::cerr << "current representation uses the following number of floats: " << std::endl;
    std::cerr << dictionary.size() * wordWidth * wordWidth << " dictionary " << std::endl;
    std::cerr << numLinearFactors << " linear factors " << std::endl;
    std::cerr << instances().size() * 12 << " for word coordinates " << std::endl;
    std::cerr << (dictionary.size() * wordWidth * wordWidth) + numLinearFactors + (instances().size() * 12) << " in total " << std::endl;
    std::cerr << std::fixed << " mean error:" << (meanError) << " " << count << std::endl;    
}

template<typename DescriptionType>
void BasicModelT<DescriptionType>::exportInstanceOrientationsAscii(std::ostream &file) const {
    //Features
    unsigned int size = _instances.size();
    for(unsigned int i = 0; i < size; i++){
	Eigen::Matrix3f m = _instances[i].pose.linear();
	for(size_t j = 0; j < 3; ++j)
	  for(size_t k = 0; k < 3; ++k)
	    file << m(j,k) << " ";
	file << std::endl;
    }
}
   
template<typename DescriptionType>
void BasicModelT<DescriptionType>::importInstanceOrientationsAscii(std::istream &file) {
    std::vector<Eigen::Matrix3f> rotations;
    while(!file.eof()){
      Eigen::Matrix3f m;
      for(size_t j = 0; j < 3; ++j)
	for(size_t k = 0; k < 3; ++k)
	  file >> m(j,k);  
      rotations.push_back(m);
    }
    std::cerr << "parsed " << rotations.size() << " rotation matrices " << std::endl;
    for(size_t i=0; i < _instances.size();++i){
      Eigen::Matrix3f rotation = _instances[i].pose.linear();
      float minRmse = 1e20;
      int rotationsId = 0;
      for(size_t j = 0; j < rotations.size();++j){
	float error = 0.0;
	for(size_t l = 0; l < 3; ++l)
	  for(size_t k = 0; k < 3; ++k)
	    error += (rotation(l,k) - rotations[j](l,k)) * (rotation(l,k) - rotations[j](l,k));
	error /= 9.f;
	error = sqrt(error);
	if(error < minRmse){
	  minRmse = error;
	  rotationsId = j;
	}
      }
      _instances[i].pose = (Eigen::Translation3f) _instances[i].pose.translation() * (Eigen::Affine3f) rotations[rotationsId];
      std::cerr << "applied rotation for instance " << i << std::endl;
    }
}
   
template<typename DescriptionType>
void BasicModelT<DescriptionType>::linkWordsAndInstances(){
  for(unsigned int j = 0; j < _dictionary.size(); j++)
    _wordMap[_dictionary[j]->id ] = _dictionary[j];

  for(unsigned int i = 0; i < _instances.size(); i++){
    _instances[i].word = _wordMap[_instances[i].fid];
    if(_instances[i].word == 0){
      std::cerr << "No feature found for element " << i << " with id " << _instances[i].fid << std::endl;
    }
    }
  }

template<typename DescriptionType>
void BasicModelT<DescriptionType>::stats(){
  unsigned int pointCount = 0;
  if(_dictionary.size() > 0)
  {
    pcl::Narf& feature = _dictionary[0]->feature;
    double featureWorldSize = feature.getSurfacePatchWorldSize();
    unsigned int featurePixelSize = feature.getDescriptorSize();
    std::cerr << "feature size: " << featureWorldSize << " descriptor size: " << featurePixelSize << std::endl;
    for(unsigned int i = 0; i < _dictionary.size(); i++){ 
        pointCount += _dictionary[i]->pointCloud.size();
    }
  }
  std::cout << "Dictionary with "<< _dictionary.size() << " words and " << pointCount << " points." << std::endl;
  std::cout << "Word instances in model " << _instances.size() << " poses."  << std::endl;
  std::cout << "theoreticall minimum size: " << _instances.size() * 7 * 4 + pointCount * 12 << " bytes" << std::endl;
}
/*
template<typename DescriptionType>
BasicModelT<DescriptionType>::importDictionary(std::vector<BasicWordT<DescriptionType>*>& dictionary)
{
  for(unsigned int j = 0; j < _instances.size(); j++){
    int id = getBasicWordT<DescriptionType>IdForInstance(_instances[j], dictionary);
    //int id = getBasicWordT<DescriptionType>IdForInstanceMaximumLikelihood(_instances[j], dictionary);
    BasicWordT<DescriptionType>* wordForInstance = dictionary[id];
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

template<typename DescriptionType>
BasicModelT<DescriptionType>::sortDictionary(){
  countWordOccurances();
  sort(_dictionary.begin(), _dictionary.end(), BasicWordT<DescriptionType>::BasicWordT<DescriptionType>Cmp_);
}*/



/*
template<typename DescriptionType>
BasicModelT<DescriptionType>::sparseInstances(){ 
  double support_size = _dictionary[0]->feature.getDescriptorSize();
  //Sparse instances
  SPModel::pclPointCloud instance_cloud;
  instancesToPointCloud(_instances, instance_cloud);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (instance_cloud.makeShared ());

  std::map< int, bool> removeInstance;
  for(size_t i = 0; i < instance_cloud.size(); ++i)
    removeInstance[i] = false;

  for(int i = instance_cloud.size()-1; i >= 0 ; --i){
    if(removeInstance[i])
      continue;
    vector<int> k_indices;
    vector<float> k_distances;
    kdtree.radiusSearch (instance_cloud, i, (support_size * 0.3), k_indices, k_distances);
    for(size_t j = 0; j < k_indices.size(); ++j){
      unsigned int id = k_indices[j];
      double desDistance = _instances[i].word->feature.getDescriptorDistance(_instances[id].word->feature);
      if(desDistance < 0.05 && id < i){
            removeInstance[id] = true;
      }
    }
  }

  for(size_t i = 0; i < _instances.size(); ++i){
    if(removeInstance[i])
      _instances[i].deleted = true;
  }
  deleteMarkedInstances();
}*/

template<typename DescriptionType>
std::vector<double> BasicModelT<DescriptionType>::getWordHistogram(std::vector<BasicWordT<DescriptionType>* >& dictionary){
  std::vector<double> histogram;
//   for(size_t i = 0; i < dictionary.size(); ++i){
//     histogram.push_back(0);
//   }
// 
//   histogram.push_back(0); 
//   int count = 0;
//   for(size_t i = 0; i < _instances.size(); i++){
//     BasicInstanceT<DescriptionType> & instance = _instances[i];
//     int id = (int) getWordIdForInstance(instance, dictionary);
//     histogram[id] += 1.0;
//     count++;
//   }
//   //Normalizing
//   for(unsigned int i = 0; i < histogram.size(); i++){
//     histogram[i] =  histogram[i] / (double) count;
//   }
//   histogram[0] = (double) count;
  return histogram;
}

