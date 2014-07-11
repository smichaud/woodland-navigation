#include "pointcloud.h"

using namespace PointMatcherSupport;

PointCloud::PointCloud() : dataPoints(){
}

PointCloud::PointCloud(PM::DataPoints pointcloud) : dataPoints(pointcloud){
}

uli PointCloud::getNbPoints() const {
    return dataPoints.features.cols();
}

PM::DataPoints &PointCloud::getDataPointsRepresentationRef() {
    PM::DataPoints &datapoints = this->dataPoints;
    return datapoints;
}

unsigned int PointCloud::getNbFeatures() const {
    return static_cast<unsigned int>(dataPoints.featureLabels.size());
}

bool PointCloud::featureExists(std::string featureName) const{
    return dataPoints.featureExists(featureName);
}

std::string PointCloud::getFeatureName(unsigned int index) const {
    this->verifyFeatureIndex(index);

    return dataPoints.featureLabels[index].text;
}

Matrix &PointCloud::getFeaturesRef() {
    Matrix & features = this->dataPoints.features;
    return features;
}

//TODO make sure the head(3) are indeed x,y,z
Vector3 PointCloud::getPoint(uli index) const {
    this->verifyPointIndex(index);
    return dataPoints.features.col(index).head(3);
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
    this->verifyDescriptorIndex(index);
    return dataPoints.descriptorLabels[index].text;
}

VectorX PointCloud::getDescriptorValue(std::string descriptorName,
                                       uli index) const {
    this->verifyDescriptorName(descriptorName);
    this->verifyPointIndex(index);

    unsigned int descriptorStartingRow =
            this->dataPoints.getDescriptorStartingRow(descriptorName);
    unsigned int descriptorSize = this->getDescriptorSize(descriptorName);

    return this->dataPoints.descriptors.block(
                descriptorStartingRow, index, descriptorSize, 1);
}

void PointCloud::setDescriptorValue(const std::string descriptorName,
                                    const uli index,
                                    const VectorX descriptorValue) {
    this->verifyPointIndex(index);
    this->verifyDescriptorName(descriptorName);
    verifyDescriptorSize(descriptorValue, descriptorName);

    unsigned int descriptorStartingRow =
            dataPoints.getDescriptorStartingRow(descriptorName);
    uli descriptorSize = descriptorValue.size();
    dataPoints.descriptors.block(descriptorStartingRow, index,
                                 descriptorSize, 1) = descriptorValue;
}

Matrix &PointCloud::getDescriptorsRef() {
    Matrix & descriptors = this->dataPoints.descriptors;
    return descriptors;
}

void PointCloud::addDescriptorInitToZero(const std::string descriptorName,
                                         int descriptorSize) {
    uli nbPoints = this->getNbPoints();
    this->dataPoints.addDescriptor(descriptorName,
                                   PM::Matrix::Zero(descriptorSize, nbPoints));
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

void PointCloud::addDescriptorRGBA(const used_type red,
                                   const used_type green,
                                   const used_type blue, used_type alpha) {
    this->verifyColor(alpha, red, green, blue);

    VectorX colors;
    colors.resize(4);
    colors << red, green, blue, alpha;

    this->addDescriptorInitToVector("color", colors);
}

void PointCloud::addObservationDirectionDescriptors(
        const used_type xSensorPosition,
        const used_type ySensorPosition,
        const used_type zSensorPosition) {
    PM::DataPointsFilter* filter(
                PM::get().DataPointsFilterRegistrar.create(
                    "ObservationDirectionDataPointsFilter",
                    map_list_of
                    ("x", toParam(xSensorPosition))
                    ("y", toParam(ySensorPosition))
                    ("z", toParam(zSensorPosition))));

    filter->inPlaceFilter(this->dataPoints);
}

void PointCloud::addKnnEigenRelatedDescriptors(unsigned int knnUsed,
                                               bool keepSurfaceNormals,
                                               bool keepDensities,
                                               bool keepEigenValues,
                                               bool keepEigenVectors) {
    PM::DataPointsFilter* filter(
                PM::get().DataPointsFilterRegistrar.create(
                    "SurfaceNormalDataPointsFilter",
                    map_list_of
                    ("knn", toParam(knnUsed))
                    ("epsilon", toParam(0))
                    ("keepNormals", toParam(keepSurfaceNormals))
                    ("keepDensities", toParam(keepDensities))
                    ("keepEigenValues", toParam(keepEigenValues))
                    ("keepEigenVectors", toParam(keepEigenVectors))
                    ("keepMatchedIds", toParam(0))));

    filter->inPlaceFilter(this->dataPoints);
}

void PointCloud::addSurfaceNormalDescriptors(unsigned int knnUsed) {
    this->addKnnEigenRelatedDescriptors(knnUsed, 1, 0, 0, 0);
}

void PointCloud::addDensityDescriptors(unsigned int knnUsed) {
    this->addKnnEigenRelatedDescriptors(knnUsed, 0, 1, 0, 0);
}

void PointCloud::addEigenDescriptors(unsigned int knnUsed) {
    this->addKnnEigenRelatedDescriptors(knnUsed, 0, 0, 1, 1);
}

void PointCloud::orientSurfaceNormalsRelativeToCenter(bool towardCenter) {
    PM::DataPointsFilter* filter(
                PM::get().DataPointsFilterRegistrar.create(
                    "OrientNormalsDataPointsFilter",
                    map_list_of("towardCenter", toParam(towardCenter))));

    filter->inPlaceFilter(this->dataPoints);
}

void PointCloud::save(const std::string filename) const{
    this->verifySaveCompatibility();
    this->dataPoints.save(filename);
}

void PointCloud::verifySaveCompatibility() const {
    int nbDescriptors = this->getNbDescriptors();
    for(int i = 0; i < nbDescriptors ; ++i) {
        int descriptorSize = this->getDescriptorSize(
                    this->getDescriptorName(i));
        std::string descriptorName = this->getDescriptorName(i);

        verifyUnsavedDescriptor(descriptorSize, descriptorName);
    }
}

void PointCloud::verifyUnsavedDescriptor(int descriptorSize,
                                          std::string descriptorName) const {
    if(!(descriptorSize == 1 || descriptorSize == 3 || descriptorSize == 9
         || (descriptorSize == 4
             && descriptorName.compare("color") == 0))) {
        std::cerr << "Descriptor (" << descriptorName
                  << ") does not respect any usual descriptor format "
                     "and will not be saved." << std::endl;
    }
}

void PointCloud::verifyPointIndex(uli index) const {
    if (index >= this->getNbPoints()) {
        throw std::out_of_range("Point index out of range");
    }
}

void PointCloud::verifyFeatureIndex(unsigned int index) const {
    if(index >= this->getNbFeatures()) {
        throw std::out_of_range("Feature index out of range");
    }
}

void PointCloud::verifyDescriptorIndex(unsigned int index) const {
    if(index >= this->getNbDescriptors()) {
        throw std::out_of_range( "Descriptor index out of range");
    }
}

void PointCloud::verifyDescriptorName(std::string descriptorName) const {
    if(!this->descriptorExists(descriptorName)) {
        throw std::invalid_argument("No such descriptor : " + descriptorName);
    }
}

void PointCloud::verifyDescriptorSize(const VectorX descriptorValue,
                                      const std::string descriptorName) const {
    if(this->getDescriptorSize(descriptorName) != descriptorValue.size()) {
        throw std::length_error("The point cloud descriptor is not the same "
                                "size as the provided descriptor vector");
    }
}

void PointCloud::verifyColor(used_type alpha, const used_type red,
                             const used_type green,
                             const used_type blue) const {
    if(red < 0 || red > 1 || green < 0 || green > 1 || blue < 0 || blue > 1
            || alpha < 0 || alpha > 1) {
        throw std::invalid_argument("RGBA values must be between 0 and 1");
    }
}


