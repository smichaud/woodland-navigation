#include <iostream>
#include "pointmatcher/PointMatcher.h"
#include "boost/timer.hpp"

#include "display.h"
#include "util.h"
#include "voxelcloud.h"

using namespace std;
typedef PointMatcher<float> PM;

int main(int argc, char *argv[])
{
    if (argc < 3)    {
        cerr << "Please enter the arguments : inputFile outputFile " << endl;
        return 1;
    }

    boost::posix_time::ptime startTime =
            boost::posix_time::second_clock::local_time();

    PM::DataPoints inputDataPoints(PM::DataPoints::load(argv[argc-2]));

    float voxelSize = 1.0;
    VoxelCloud voxelCloud(inputDataPoints, voxelSize);

    PM::DataPoints outputDataPoints = voxelCloud.getSinglePointPerVoxel();
    outputDataPoints.save(argv[argc-1]);

    boost::posix_time::ptime endTime =
            boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff = endTime - startTime;
    cout << "Time elapsed (ms) : " << diff.total_milliseconds() << endl;

    return 0;
}
