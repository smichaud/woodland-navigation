import os
import sys
import glob

from pointcloud_convertion import pcd_to_csv
from scene import Scene
from configuration import Configuration

def main():
    # config_filename = "ConfigFiles/config.json"
    # configuration = Configuration()
    # scene = Scene()
    # scene.cylinder_radius = 0.05
    # scene.cylinder_count = 5
    # configuration.add_scene(scene)
    #
    # scene.cylinder_radius = 0.1
    # scene.cylinder_count = 1
    # configuration.add_scene(scene)
    #
    # conf = Configuration()
    # conf.from_json(configuration.to_json())
    # print configuration.scenes
    # print conf.scenes


    blensor = '/home/smichaud/blensor-1.0.16rc1/build/bin/blender'
    pythonScript = '/home/smichaud/Workspace/WoodlandNavigation/Utils/PointCloudGeneration/blensor_script.py'

    print("Starting the point clouds generation...\n")

    os.system(blensor + ' -b -P ' + pythonScript)
    for file in glob.glob("PointClouds/*.pcd"):
        pcd_to_csv(file)

    print("\nJob done !")

if __name__ == "__main__":
    main()