#include <iostream>
#include "pointmatcher/PointMatcher.h"
#include "boost/timer.hpp"

#include "definitions.h"
#include "display.h"
#include "voxelgridpointcloud.h"
#include "groundprocessing.h"
#include "mathutil.h"

using namespace std;

void usage() {
    cerr << "Please enter the arguments : inputFile outputFile " << endl;
}

int main(int argc, char *argv[])
{
    if (argc < 3) {
        usage();
        return 1;
    }

    boost::posix_time::ptime startTime =
            boost::posix_time::second_clock::local_time();
    ////////////////////////////////////////////////////////////////////////////

    PointCloud pointCloud(PM::DataPoints::load(argv[argc-2]));
    pointCloud.addDescriptorInitToVector("color",
                                         Eigen::Matrix<float,2,1>::Zero());
    pointCloud.save(argv[argc-1]);

//    VoxelGridPointCloud voxelCloud(pointCloud, 3,3,0);

//    GroundProcessing::addGroundDescriptor(voxelCloud);
//    voxelCloud.getPointCloudRef().save(argv[argc-1]);


    ////////////////////////////////////////////////////////////////////////////
    boost::posix_time::ptime endTime =
            boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff = endTime - startTime;
    cout << endl << "Time elapsed (ms) : " << diff.total_milliseconds() << endl;

    return 0;
}
