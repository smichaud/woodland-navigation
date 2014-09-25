#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <geometry_msgs/Vector3Stamped.h>

using namespace std;

class RollPitchYawExtractor {
private:
    vector<ros::Time> timestamps;
    vector<double> roll;
    vector<double> pitch;
    vector<double> yaw;

    ros::Subscriber subscriber;
    ros::NodeHandle subscriberNodeHandle;
    const string subscribeTopic;

public:
    RollPitchYawExtractor(ros::NodeHandle& subscriberNodeHandle):
        subscriberNodeHandle(subscriberNodeHandle),
        subscribeTopic("/imu/rpy"){
        subscriber = this->subscriberNodeHandle.subscribe(
                    subscribeTopic, 1000,
                    &RollPitchYawExtractor::gotMeasurement, this);
    }
    void gotMeasurement(const geometry_msgs::Vector3Stamped imuRollPitchYaw){
        roll.push_back(imuRollPitchYaw.vector.x);
        pitch.push_back(imuRollPitchYaw.vector.y);
        yaw.push_back(imuRollPitchYaw.vector.z);
        timestamps.push_back(imuRollPitchYaw.header.stamp);
    }
    void saveCSV(const string fileName){
        ofstream outputFile;
        outputFile.open(fileName);
        if(!outputFile.is_open()) {
            cout << "Impossible to write to: " << fileName << endl;
        } else {
            cout << endl << "Saving data as: " << fileName << "..." << endl;
            for(unsigned i = 0 ; i < this->timestamps.size(); ++i) {
                ros::Duration timeFromBeginning = this->timestamps[i]
                        - this->timestamps[0];
                outputFile << timeFromBeginning.toSec()
                           << ", " << this->roll[i]
                              << ", " << this->pitch[i]
                                 << ", " << this->yaw[i] << endl;
            }
        }
    }
    bool haveData(){
        return !(this->timestamps.empty());
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "roll_pitch_yaw_extractor");

    ros::NodeHandle nodeHandle("~");
    string fileName;
    nodeHandle.param<string>("output", fileName, "~/.ros/roll_pitch_yaw.csv");
    cout << fileName << endl;

    RollPitchYawExtractor rollPitchYawExtractor(nodeHandle);

    ros::spin();

    if(rollPitchYawExtractor.haveData()){
        rollPitchYawExtractor.saveCSV(fileName);
    } else {
        cout << endl << "No data to save ..." <<  endl;
    }

    return 0;
}



