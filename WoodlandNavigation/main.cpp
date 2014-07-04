#include <iostream>
#include "pointmatcher/PointMatcher.h"
#include "boost/timer.hpp"

#include "definitions.h"
#include "display.h"
#include "huskya200.h"
#include "pointcloudfilter.h"

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
    PointCloud testPointCloud(PM::DataPoints::load(argv[argc-2]));

    testPointCloud.addDensityDescriptors(20);

    PointCloudFilter::minRadius(testPointCloud, 1.2f);
    PointCloudFilter::maxRadius(testPointCloud, 15);
    PointCloudFilter::maxDensity(testPointCloud, 10);

    testPointCloud.addObservationDirectionDescriptors();
    testPointCloud.addSurfaceNormalDescriptors(75);
    testPointCloud.orientSurfaceNormalsRelativeToCenter();

    HuskyA200 husky;
    husky.gotPointCloud(testPointCloud);
    husky.savePointCloudRepresentation(argv[argc-1]);
    ////////////////////////////////////////////////////////////////////////////

    boost::posix_time::ptime endTime =
            boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff = endTime - startTime;
    cout << endl << "Time elapsed (ms) : " << diff.total_milliseconds() << endl;

    return 0;
}
