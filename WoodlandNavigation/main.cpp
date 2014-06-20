#include <iostream>
#include "pointmatcher/PointMatcher.h"
#include "boost/timer.hpp"

#include "definitions.h"
#include "display.h"
#include "voxelgridpointcloud.h"
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

    PM::DataPoints inputDataPoints(PM::DataPoints::load(argv[argc-2]));

    //    Display::printAllInfo(inputDataPoints);
    VoxelGridPointCloud voxelCloud(inputDataPoints,0,0,0);
//    Eigen::Matrix4f test;
//    test << 1,2,3,4, 5,6,7,8, 9,10,11,12, 13,14,15,16;
    cout << voxelCloud.getNbOfVoxels() << endl;
    cout << voxelCloud.getVoxelSize() << endl;

//    cout << test.col(3).head(3) << endl;


    //    PM::DataPoints outputDataPoints = voxelCloud.getSinglePointPerVoxel();
    //    outputDataPoints.save(argv[argc-1]);

    ////////////////////////////////////////////////////////////////////////////
    boost::posix_time::ptime endTime =
            boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff = endTime - startTime;
    cout << endl << "Time elapsed (ms) : " << diff.total_milliseconds() << endl;

    return 0;
}
