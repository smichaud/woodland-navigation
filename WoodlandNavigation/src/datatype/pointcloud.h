#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "definitions.h"
#include <iostream>

// This is mainly a user personnal interface of the PointMatcher Datapoints
class PointCloud
{
private:
    PM::DataPoints dataPoints;

public:
    PointCloud();
    PointCloud(PM::DataPoints pointcloud);

    uli getNbPoints() const;

    unsigned int getNbFeatures() const;
    bool featureExists(std::string featureName) const;
    std::string getFeatureName(unsigned int index) const;
    Matrix &getFeaturesRef();

    unsigned int getNbDescriptors() const;
    unsigned int getDescriptorSize(const std::string descriptorName) const;
    bool descriptorExists(const std::string descriptorName) const;
    std::string getDescriptorName(unsigned int index) const;
    Matrix &getDescriptorsRef();


    void addDescriptorInitToZero(const std::string descriptorName,
                                 int nbValues=1);
    void addDescriptorInitToVector(const std::string descriptorName,
                                   const VectorX descriptorDefaultValue);
    void addDescriptor(const std::string descriptorName,
                       const PM::Matrix descriptors);
    void appendDescriptorRGBA(used_type red, used_type green, used_type blue,
                              used_type alpha);
};

#endif
