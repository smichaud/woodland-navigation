#include "pointcloud.h"

PointCloud::PointCloud() : dataPoints(){
}

PointCloud::PointCloud(PM::DataPoints pointcloud) : dataPoints(pointcloud){
}

uli PointCloud::getNbPoints() const {
    return dataPoints.features.cols();
}

unsigned int PointCloud::getNbFeatures() const {
    return static_cast<unsigned int>(dataPoints.featureLabels.size());
}

bool PointCloud::featureExists(std::string featureName) const{
    return dataPoints.featureExists(featureName);
}

std::string PointCloud::getFeatureName(unsigned int index) const {
    if(index >= this->getNbFeatures()) {
        std::string msg = "Feature index out of range";
        throw std::invalid_argument(msg);
    } else {
        return dataPoints.featureLabels[index].text;
    }
}

Matrix &PointCloud::getFeaturesRef() {
    Matrix & features = this->dataPoints.features;
    return features;
}

unsigned int PointCloud::getNbDescriptors() const {
    return static_cast<unsigned int>(dataPoints.descriptorLabels.size());
}

unsigned int PointCloud::getDescriptorSize(
        const std::string descriptorName) const {
    return dataPoints.getDescriptorDimension(descriptorName);
}

bool PointCloud::descriptorExists(const std::string descriptorName) const {
    return dataPoints.descriptorExists(descriptorName);
}

std::string PointCloud::getDescriptorName(unsigned int index) const {
    if(index >= this->getNbDescriptors()) {
        std::string msg = "Descriptor index out of range";
        throw std::invalid_argument(msg);
    } else {
        return dataPoints.descriptorLabels[index].text;
    }
}

Matrix &PointCloud::getDescriptorsRef() {
    Matrix & descriptors = this->dataPoints.descriptors;
    return descriptors;
}

void PointCloud::addDescriptorInitToZero(const std::string descriptorName,
                                         int nbValues) {
    uli nbPoints = this->getNbPoints();
    this->dataPoints.addDescriptor(descriptorName,
                                   PM::Matrix::Zero(nbValues, nbPoints));
}

void PointCloud::addDescriptorInitToVector(
        const std::string descriptorName,
        const VectorX descriptorDefaultValue) {
    uli nbValuesPerPoint = descriptorDefaultValue.size();
    PM::Matrix descriptors;
    descriptors.setOnes(nbValuesPerPoint, this->getNbPoints());

    for(uli i = 0; i < nbValuesPerPoint; ++i) {
        descriptors.row(i) = descriptors.row(i)*descriptorDefaultValue[i];
    }

    this->dataPoints.addDescriptor(descriptorName, descriptors);
}

void PointCloud::addDescriptor(const std::string descriptorName,
                               const PM::Matrix descriptors) {
    dataPoints.addDescriptor(descriptorName, descriptors);
}

void PointCloud::appendDescriptorRGBA(used_type red, used_type green,
                                      used_type blue, used_type alpha) {
    if(red < 0 || red > 1 || green < 0 || green > 1 || blue < 0 || blue > 1
            || alpha < 0 || alpha > 1) {
        std::string msg = "RGBA values must be between 0 and 1";
        throw std::invalid_argument(msg);
    }
    VectorX colors;
    colors.resize(4);
    colors << red, green, blue, alpha;

    this->addDescriptorInitToVector("color", colors);
}
