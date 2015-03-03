#!/usr/bin/python
import os, sys, glob, re

files = glob.glob("./ExtractedData/*_point_cloud_0.csv");

for currentFile in files:
    filePath = os.path.split(currentFile)[0];
    fileName = os.path.split(currentFile)[1];
    fileBaseName = os.path.splitext(currentFile)[0];
    filePrefix = re.sub('\_point_cloud_0$', '', fileBaseName);
     
    programCommand = "/home/smichaud/Workspace/libpointmatcher/build/examples/pmicp ";
    isTransfoSavedCommand = "--isTransfoSaved true ";
    initialTranslationCommand = "--initTranslation [4,0,0] ";
    initialRotationCommand = "--initRotation [1,0,0\;0,1,0\;0,0,1] "
    configCommand = "--config config.yaml ";
    outputCommand = "--output " + filePrefix + " ";
    referenceDataPoints = filePrefix + "_point_cloud_0.csv ";
    readingDataPoints = filePrefix + "_point_cloud_1.csv ";
    # Note : If a point was at a certain position in point_cloud_0, it will be 4 meters behind in point_cloud_0
    # Therefore, to have odometry, use point_cloud_0 as reference (point_cloud_1 + 4m fits point_cloud_0)
    
    command = programCommand + isTransfoSavedCommand + initialTranslationCommand + initialRotationCommand + configCommand + outputCommand + referenceDataPoints + readingDataPoints;    
    os.system(command);
    
    # To remove extra (useless?) files
    os.system("rm " + filePrefix + "_data_in.vtk");
    #os.system("rm " + filePrefix + "_data_out.vtk");
    #os.system("rm " + filePrefix + "_ref.vtk");
    os.system("rm " + filePrefix + "_init_transfo.txt");
    
print "Done ! \n"
