#include "display.h"

using namespace std;

namespace Display {

void printAllInfo(const PointCloud &pointcloud)
{
    cout << "All informations:" << endl;
    printNumberOfPoints(pointcloud);
    printFeatures(pointcloud);
    printDescriptors(pointcloud);
}

void printNumberOfPoints(const PointCloud &pointcloud)
{
    cout << "----- Number of points " << endl;
    cout << "Number of points/columns: "
         << pointcloud.getNbPoints() << endl;
    cout << "-------------------------" << endl;
}

void printFeatures(const PointCloud &pointcloud)
{
    cout << "----- Features " << endl;
    cout << "Number of features: " << pointcloud.getNbFeatures()
         << endl;
    cout << "Feature labels : " << endl;
    for(unsigned int i=0 ; i < pointcloud.getNbFeatures() ; ++i){
        cout << "   - " << pointcloud.getFeatureName(i) << endl;
    }
    cout << "-------------------------" << endl;
}

void printDescriptors(const PointCloud &pointcloud)
{
    cout << "----- Descriptors " << endl;
    cout << "Number of descriptors: " << pointcloud.getNbDescriptors()
         << endl;
    cout << "Descriptor labels: " << endl;
    for(unsigned int i = 0 ; i < pointcloud.getNbDescriptors() ;
        ++i){
        std::string descriptorName = pointcloud.getDescriptorName(i);
        cout << "   - " << descriptorName
             << " (of size "
             << pointcloud.getDescriptorSize(descriptorName) << ")"
             << endl;
    }
    cout << "-------------------------" << endl;
}

}
