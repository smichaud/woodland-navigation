#!/usr/bin/python

# This is a quick script manually adjusted for my acquisitions

import os, sys, glob, time

timeForAScan = 27;

commandPrefix = "nohup " # Prevent program hang-up
commandSuffix = " > /dev/null 2>&1&" # Prevent all command output

print "===== Start to record the bag file"
os.system(commandPrefix + "rosbag record -a" + commandSuffix);
time.sleep(0.5)

print "===== Scanning initial point cloud"
os.system(commandPrefix + "roslaunch ptu_laser_assembler singleAcquisitionHighDensity.launch" + commandSuffix);
time.sleep(timeForAScan)

print "===== Moving straight forward"
accelerationTreshold = 0.2; # or is it communication ?
speed = 0.3; # m/s
distance = 4; # meters
command = "rostopic pub /husky/cmd_vel geometry_msgs/Twist -r 10 -- '[" + str(speed) + ", 0.0, 0.0]' '[0.0, 0.0, 0.0]'";
os.system(commandPrefix + command + commandSuffix);
time.sleep(distance/speed + accelerationTreshold)
os.system("pkill -SIGINT rostopic")

print "===== Scanning final point cloud"
os.system(commandPrefix + "roslaunch ptu_laser_assembler singleAcquisitionHighDensity.launch" + commandSuffix);
time.sleep(timeForAScan)

time.sleep(1)
os.system("pkill -SIGINT record")
time.sleep(1)

print "\nSample saved !"
