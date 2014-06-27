#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "definitions.h"
#include <iostream>

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
    Vector3 getPoint(uli index) const;

    unsigned int getNbDescriptors() const;
    unsigned int getDescriptorSize(const std::string descriptorName) const;
    bool descriptorExists(const std::string descriptorName) const;
    std::string getDescriptorName(unsigned int index) const;
    Matrix &getDescriptorsRef();


    void addDescriptorInitToZero(const std::string descriptorName,
                                 int descriptorSize=1);
    void addDescriptorInitToVector(const std::string descriptorName,
                                   const VectorX descriptorDefaultValue);
    void addDescriptor(const std::string descriptorName,
                       const PM::Matrix descriptors);
    void addDescriptorRGBA(const used_type red,
                              const used_type green,
                              const used_type blue,
                              used_type alpha);
    void setDescriptorValue(const std::string descriptorName, const uli index,
                            const VectorX descriptorValue);

    void save(const std::string filename) const;
};

#endif
