#include "pointcloudprocessing.h"

#include <stdlib.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <fstream>

#include "stdafx.h"
#include "dataanalysis.h"

using namespace std;

namespace PointCloudProcessing {

// TODO: alglib makes me want to puke, I should make my own implementation
// of the single linkage hierarchical clustering
// It takes 12 seconds for 10k points, and crashes if there is too much points
void labelSingleLinkageClusters(PointCloud &pointcloud,
                                std::string labelName,
                                std::vector<uli> indexes,
                                used_type maxDist) {
    pointcloud.addDescriptorInitToZero(labelName);
    auto nbPoints = indexes.size();
    VectorX labels(nbPoints);

    stringstream serializationStringStream;
    serializationStringStream << "[";
    for(auto i : indexes) {
        serializationStringStream << "["
                                  << pointcloud.getPoint(i)[xIndex] <<  ","
                                  << pointcloud.getPoint(i)[yIndex] <<  ","
                                  << pointcloud.getPoint(i)[zIndex] <<  "],";
    }
    string serializedPointcloud = serializationStringStream.str();
    serializedPointcloud[serializedPointcloud.size()-1] = ']';


    alglib::clusterizerstate clusterizerstate;
    alglib::ahcreport ahcreport;
    alglib::real_2d_array points = serializedPointcloud.c_str();

    alglib::clusterizercreate(clusterizerstate);
    alglib::clusterizersetpoints(clusterizerstate, points, 2);

    const alglib::ae_int_t singleLinkage = 1;
    alglib::clusterizersetahcalgo(clusterizerstate, singleLinkage);
    alglib::clusterizerrunahc(clusterizerstate, ahcreport);

    alglib::ae_int_t nbCluster = 0;
    alglib::integer_1d_array pointsLabels;
    alglib::integer_1d_array clusterIndexMapRepZ;
    alglib::clusterizerseparatedbydist(ahcreport, maxDist, nbCluster,
                                       pointsLabels, clusterIndexMapRepZ);


    stringstream deserializationStringStream(pointsLabels.tostring());
    int convertedLabel = 0;
    string label;
    uli i = 0;
    while( getline(deserializationStringStream, label, ',') ) {
        istringstream(label) >> convertedLabel;
        pointcloud.setDescriptorValue(labelName, indexes[i], Scalar::Ones()*
                                      static_cast<used_type>(
                                          convertedLabel+1));
        ++i;
    }
}


}

