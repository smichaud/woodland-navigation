#include "clearpath_base/SystemStatus.h"
#include "ros/ros.h"
#include <iostream>
#include <fstream>

using namespace std;

class HuskyMotorCurrents {
private:
    vector<ros::Time> timestamps;
    vector<double> leftCurrents;
    vector<double> rightCurrents;

    ros::Subscriber subscriber;
    ros::NodeHandle subscriberNodeHandle;
    const string subscribeTopic;

public:
    HuskyMotorCurrents(ros::NodeHandle& subscriberNodeHandle):
        subscriberNodeHandle(subscriberNodeHandle),
        subscribeTopic("/husky/data/system_status"){
        subscriber = this->subscriberNodeHandle.subscribe(
                    subscribeTopic, 1000,
                    &HuskyMotorCurrents::gotCurrent, this);
    }
    void gotCurrent(const clearpath_base::SystemStatus huskyStatus){
        leftCurrents.push_back(huskyStatus.currents.at(1));
        rightCurrents.push_back(huskyStatus.currents.at(2));
        timestamps.push_back(huskyStatus.header.stamp);
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
                           << ", " << this->leftCurrents[i]
                              << ", " << this->rightCurrents[i] << endl;
            }
        }
    }
    bool haveData(){
        return !(this->timestamps.empty());
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_currents");

    ros::NodeHandle nodeHandle("~");
    string fileName;
    nodeHandle.param<string>("output", fileName, "~/.ros/motor_currents.csv");
    cout << fileName << endl;

    HuskyMotorCurrents huskyMotorCurrents(nodeHandle);

    ros::spin();

    if(huskyMotorCurrents.haveData()){
        huskyMotorCurrents.saveCSV(fileName);
    } else {
        cout << endl << "No data to save ..." <<  endl;
    }

    return 0;
}


