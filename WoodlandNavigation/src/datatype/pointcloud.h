#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "definitions.h"
#include <iostream>

class PointCloud
{
public:
    PM::DataPoints dataPoints;

public:
    PointCloud();
    PointCloud(PM::DataPoints pointcloud);

    uli getNbPoints() const;

    PM::DataPoints &getDataPointsRepresentationRef();

    unsigned int getNbFeatures() const;
    bool featureExists(std::string featureName) const;
    std::string getFeatureName(unsigned int index) const;
    Matrix &getFeaturesRef();
    Vector3 getPoint(uli index) const;

    unsigned int getNbDescriptors() const;
    unsigned int getDescriptorSize(const std::string descriptorName) const;
    bool descriptorExists(const std::string descriptorName) const;
    std::string getDescriptorName(unsigned int index) const;

    VectorX getDescriptorValue(std::string descriptorName, uli index) const;
    void setDescriptorValue(const std::string descriptorName, const uli index,
                            const VectorX descriptorValue);
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

    void addObservationDirectionDescriptors(
            const used_type xSensorPosition = 0,
            const used_type ySensorPosition = 0,
            const used_type zSensorPosition = 0);
    void addKnnEigenRelatedDescriptors(unsigned int knnUsed = 5,
                                       bool keepSurfaceNormals = true,
                                       bool keepDensities = true,
                                       bool keepEigenValues = true,
                                       bool keepEigenVectors = true);
    void addSurfaceNormalDescriptors(unsigned int knnUsed = 5);
    void addDensityDescriptors(unsigned int knnUsed = 5);
    void addEigenDescriptors(unsigned int knnUsed = 5);
    void orientSurfaceNormalsRelativeToCenter(bool towardCenter = true);

    void save(const std::string filename) const;

private:
    void verifySaveCompatibility() const;
    void verifyUnsavedDescriptor(int descriptorSize,
                                 std::string descriptorName) const;

    void verifyPointIndex(uli index) const;
    void verifyFeatureIndex(unsigned int index) const;
    void verifyDescriptorIndex(unsigned int index) const;
    void verifyDescriptorName(std::string descriptorName) const;
    void verifyDescriptorSize(const VectorX descriptorValue,
                              const std::string descriptorName) const;

    void verifyColor(used_type alpha, const used_type red,
                     const used_type green, const used_type blue) const;
};

#endif
