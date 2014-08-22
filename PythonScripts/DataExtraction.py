#!/usr/bin/python

import os, sys, glob, time

if len(sys.argv) < 2:
    print "DataExtraction require a single argument: the path to the directory containing all bag files"
    sys.exit()
else:
    bagFolder = sys.argv[1]
    print "Start extracting data from : " + bagFolder

#os.system("nohup roscore &")
#time.sleep(1) # Let roscore start properly

bagFiles = glob.glob(bagFolder + "/*.bag")
for bagFile in bagFiles:    
    bagFileBaseName = os.path.splitext(bagFile)[0]
    
    if not os.path.exists(bagFileBaseName):
        os.makedirs(bagFileBaseName)
        
    #start image saver
    #start pointcloud saver
    #start motor currents saver
    
#    os.system("rosbag play --clock " + bagFile)

#os.system("pkill -SIGINT roscore")

print "\nExtracted data will be in the directories named after the bag files."
