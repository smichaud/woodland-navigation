#!/usr/bin/python3

import string, sys, os

inputFileString = sys.argv[1]
inputFilePath = os.path.split(inputFileString)[0] + '/'
inputFileName = os.path.split(inputFileString)[1]
inputFileBaseName = os.path.splitext(inputFileString)[0]

inputFile = inputFilePath + inputFileName
outputFile = inputFileBaseName + '.csv'

outputHandle = open(outputFile, "w")
with open(inputFile) as inputHandle:
    for line in inputHandle:
        if line[0].isdigit() or line[0] ==  '-':
            outputHandle.write(line.replace(' ', ','))
        elif line.startswith('FIELDS '):
            outputHandle.write(line.replace('FIELDS ', '').replace(' ', ','))
            
outputHandle.close()
