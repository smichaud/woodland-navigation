#!/bin/bash

blensor='/home/smichaud/blensor-1.0.16rc1/build/bin/blender'
defaultScene='/home/smichaud/Desktop/DataGeneration/generated.blend'
pythonScript='/home/smichaud/Workspace/WoodlandNavigationUtils/PythonScripts/PointCloudGeneration/BlensorGenerator.py'

$blensor -b $defaultScene -P $pythonScript
