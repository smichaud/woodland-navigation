#!/usr/bin/python
import os, sys, glob, time

commandPrefix = "nohup " # Prevent program hang-up
commandSuffix = " > /dev/null 2>&1&" # Prevent all command output

for bagFile in bagFiles:
    print "===== Start to record the bag file"
    os.system(commandPrefix + "rosbag record -a" + commandSuffix);
    time.sleep(1)
    
    print "===== Scanning initial point cloud"
    os.system(commandPrefix + "roslaunch ptu_laser_assembler singleAcquisitionHighDensity.launch" + commandSuffix);
    time.sleep(6)
    
    print "===== Moving straight forward (4 meters)"
    os.system(commandPrefix + "" + commandSuffix);
    time.sleep()
    
    print "===== Scanning final point cloud"
    os.system(commandPrefix + "roslaunch ptu_laser_assembler singleAcquisitionHighDensity.launch" + commandSuffix);
    
    time.sleep(1)
    os.system("rosnode kill rosbag")
    time.sleep(1)


print "\nSample saved !"
