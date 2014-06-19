#include "display.h"

using namespace std;

void Display::printAllInfo(const PM::DataPoints &dataPoints)
{
    cout << "All informations:" << endl;
    Display::printNumberOfPoints(dataPoints);
    Display::printFeatures(dataPoints);
    Display::printDescriptors(dataPoints);
}

void Display::printNumberOfPoints(const PM::DataPoints &dataPoints)
{
    cout << "----- Number of points " << endl;
    cout << "Number of points/columns: "
         << dataPoints.features.cols() << endl;
    cout << "-------------------------" << endl;
}

void Display::printFeatures(const PM::DataPoints &dataPoints)
{
    cout << "----- Features " << endl;
    cout << "Number of features: " << dataPoints.featureLabels.size()
         << endl;
    cout << "Feature labels : " << endl;
    for(unsigned long int i=0 ; i < dataPoints.featureLabels.size() ; ++i){
        cout << "   - " << dataPoints.featureLabels[i].text << endl;
    }
    cout << "-------------------------" << endl;
}

void Display::printDescriptors(const PM::DataPoints &dataPoints)
{
    cout << "----- Descriptors " << endl;
    cout << "Number of descriptors: " << dataPoints.descriptorLabels.size()
         << endl;
    cout << "Descriptor labels: " << endl;
    for(int i=0 ; i < static_cast<int>(dataPoints.descriptorLabels.size()) ;
        ++i){
        string descriptorName = dataPoints.descriptorLabels[i].text;
        cout << "   - " << descriptorName
             << " (of size "
             << dataPoints.getDescriptorDimension(descriptorName) << ")"
             << endl;
    }
    cout << "-------------------------" << endl;
}


