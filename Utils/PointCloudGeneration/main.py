import os
import sys
import glob

from pointcloud_convertion import pcd_to_csv
from scene_generation_config import SceneGenerationConfig

def main():
    config = SceneGenerationConfig()
    config.from_json()

    blensor_command = '/home/edminster/Programs/blensor-1.0.16rc1/build/bin/blender'
    pythonScript = 'blensor_script.py'

    print("Starting the point clouds generation...\n")

    os.system(blensor_command + ' -b -P ' + pythonScript)
    for file in glob.glob("PointClouds/*.pcd"):
        pcd_to_csv(file)

    print("\nJob done !")

if __name__ == '__main__':
    main()