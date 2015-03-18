import os
import sys
import glob

from pointcloud_convertion import pcd_to_csv
from scene_generation_config import SceneGenerationConfig

def main():
    config = SceneGenerationConfig()
    config.from_json()

    for primitive in config.primitives:
        location = tuple(primitive['location']) if ('location' in primitive) else (0,0,0)
        rotation = tuple(primitive['rotation']) if ('rotation' in primitive) else (0,0,0)
        radius = primitive['radius'] if ('radius' in primitive) else 0
        depth = primitive['depth'] if ('depth' in primitive) else 0

        if 'type' in primitive:
            if primitive['type'] == 'cylinder':
                print 'cylinder'
            elif primitive['type'] == 'sphere':
                print 'sphere'
            elif primitive['type'] == 'cube':
                print 'cube'
            elif primitive['type'] == 'cone':
                print 'cone'
            elif primitive['type'] == 'torus':
                print 'torus'



    # blensor = '/home/smichaud/blensor-1.0.16rc1/build/bin/blender'
    # pythonScript = 'blensor_script.py'
    #
    # print("Starting the point clouds generation...\n")
    #
    # os.system(blensor + ' -b -P ' + pythonScript)
    # for file in glob.glob("PointClouds/*.pcd"):
    #     pcd_to_csv(file)
    #
    # print("\nJob done !")

if __name__ == '__main__':
    main()