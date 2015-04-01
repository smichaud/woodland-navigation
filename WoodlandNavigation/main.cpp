#include <iostream>
#include <pointmatcher/PointMatcher.h>
#include <boost/timer.hpp>
#include "stdafx.h"
#include "dataanalysis.h"

#include "definitions.h"
#include "display.h"

#include "placerecognitionprototype.h"


using namespace std;

bool validateArguments(int &argc) {
    if (argc < 3) {
        cerr << "Please enter the arguments : inputFile outputFile " << endl;
        return false;
    }
    return true;
}

int main(int argc, char *argv[]) {
    boost::posix_time::ptime startTime =
            boost::posix_time::second_clock::local_time();

    //Prototype here
    PlaceRecognitionPrototype prototype;
    prototype.run();

    boost::posix_time::ptime endTime =
            boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff = endTime - startTime;
    cout << endl << "Time elapsed (ms) : " << diff.total_milliseconds() << endl;

    return 0;
}
