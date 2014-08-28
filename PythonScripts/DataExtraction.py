#!/usr/bin/python
import os, sys, glob, time

if len(sys.argv) < 2:
    print "DataExtraction require a single argument: the path to the directory containing all bag files"
    sys.exit()
else:
    bagDirectory = sys.argv[1]
    print "Start extracting data from : " + bagDirectory

bagFiles = glob.glob(bagDirectory + "/*.bag")
if not os.path.exists(bagDirectory) or len(bagFiles) == 0:
    print "Nothing to do: the directory does not exist or does not contain any bag file"
    sys.exit(0)

extractedDataDirectory = bagDirectory + "/ExtractedData"
if not os.path.exists(extractedDataDirectory):
    os.makedirs(extractedDataDirectory)
    
commandPrefix = "nohup " # Prevent program hang-up
commandSuffix = " > /dev/null 2>&1&" # Prevent all command output

os.system(commandPrefix + "roscore" + commandSuffix)
time.sleep(1) # Let roscore start properly
print "roscore started..."

for bagFile in bagFiles:
    print "===== Extracting data from : " + bagFile    
    bagFilePath = os.path.split(bagFile)[0]
    bagFileName = os.path.split(bagFile)[1]
    bagFileBaseName = os.path.splitext(bagFileName)[0]
    
    print "Starting the motor current extractor node..."    
    motorCurrentsExtractorCommand = "rosrun data_extraction motor_currents_extractor "
    motorCurrentsExtractorCommand = motorCurrentsExtractorCommand + "_output:='" + extractedDataDirectory + "/" + bagFileBaseName + "_motor_currents.csv" + "'"
    os.system(commandPrefix + motorCurrentsExtractorCommand + commandSuffix);
        
    print "Starting the point cloud extractor node..."
    pointCloudExtractorCommand = "rosrun ethzasl_point_cloud_vtk_tools pointCloudToVtk "
    pointCloudExtractorCommand = pointCloudExtractorCommand + "_cloudTopic:='/cloud' "
    pointCloudExtractorCommand = pointCloudExtractorCommand + "_mapFrameId:='/base_link' "
    pointCloudExtractorCommand = pointCloudExtractorCommand + "_recordOnce:='true' "
    pointCloudExtractorCommand = pointCloudExtractorCommand + "_outputPath:='" + extractedDataDirectory + "' "
    pointCloudExtractorCommand = pointCloudExtractorCommand + "_outputExtension:=csv "
    pointCloudExtractorCommand = pointCloudExtractorCommand + "_outputPrefix:='" + bagFileBaseName + "_point_cloud' " 
    os.system(commandPrefix + pointCloudExtractorCommand + commandSuffix)
    
    print "Starting the image extractor node..."
    imageExtractorCommand = "rosrun data_extraction image_extractor "
    imageExtractorCommand = imageExtractorCommand + "_filename:='" + extractedDataDirectory + "/" + bagFileBaseName + ".jpg" "'"
    os.system(commandPrefix + imageExtractorCommand + commandSuffix);

    os.system("rosbag play --clock " + bagFile)
    
    time.sleep(1)
    os.system("rosnode kill /motor_currents_extractor")

os.system("pkill -SIGINT roscore")

print "\nExtracted data saved in: " + extractedDataDirectory
