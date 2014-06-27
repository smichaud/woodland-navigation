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
    pointCloud.addDescriptorInitToZero("test", 4);
    VoxelGridPointCloud voxelCloud(pointCloud, 1.0f,1.0f,0);
    cout << "voxelSize: " << voxelCloud.getVoxelSize().transpose() << endl;
    cout << "nbVoxels: " << voxelCloud.getNbVoxels().transpose() << endl;
    cout << "voxel 10 10 0: " << voxelCloud.getVoxel(10,10,0).getNbPoints() << endl;


//    GroundProcessing::addGroundDescriptor(voxelCloud);


    ////////////////////////////////////////////////////////////////////////////
    boost::posix_time::ptime endTime =
            boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff = endTime - startTime;
    cout << endl << "Time elapsed (ms) : " << diff.total_milliseconds() << endl;

    return 0;
}
