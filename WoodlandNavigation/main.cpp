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
    VoxelGridPointCloud voxelCloud(inputDataPoints, 0.5f,0.5f,0);
    //    outputDataPoints.save(argv[argc-1]);

    vector<indexAndValue > vect;
    vect.push_back(indexAndValue(5,0.0f));
    vect.push_back(indexAndValue(4,2.2f));
    vect.push_back(indexAndValue(3,4.4f));
    vect.push_back(indexAndValue(2,6.6f));
    vect.push_back(indexAndValue(1,8.8f));
    vect.push_back(indexAndValue(0,10.0f));

    cout << "Quantile index: "
         << MathUtil::getRoundedQuantileRelatedIndice(vect, 0.3f)
         << endl;

    ////////////////////////////////////////////////////////////////////////////
    boost::posix_time::ptime endTime =
            boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff = endTime - startTime;
    cout << endl << "Time elapsed (ms) : " << diff.total_milliseconds() << endl;

    return 0;
}
