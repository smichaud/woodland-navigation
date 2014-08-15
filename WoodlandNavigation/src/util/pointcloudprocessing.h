#ifndef POINTCLOUDPROCESSING_H
#define POINTCLOUDPROCESSING_H

#include "definitions.h"
#include "pointcloud.h"

namespace PointCloudProcessing {

void labelSingleLinkageClusters(PointCloud &pointcloud, std::string labelName,
                                std::vector<uli> indexes,
                                used_type maxDist);

}

#endif
