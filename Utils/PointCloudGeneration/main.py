import os
import glob
import math
from random import uniform
from random import normalvariate

from pointcloud_convertion import pcd_to_csv
from scene_generation_config import SceneGenerationConfig

def generate_cylinders_config(name, cylinders_count, cylinders_radius, radius_sigma=0):
    config = SceneGenerationConfig()

    prefix = "count_" + str(cylinders_count) + "_radius_mm_" + str(int(cylinders_radius*1000)) + "_"
    config.output_blend_file = "BlendFiles/" + prefix + name + ".blend"
    config.output_pcd_file = "PointClouds/" + prefix + name + ".pcd"

    cylinder_height = 3
    angle_max = math.radians(40)
    for i in range(0, cylinders_count):
        config.add_primitive({
            'type': 'cylinder',
            'radius': normalvariate(cylinders_radius, sigma=radius_sigma),
            'depth': cylinder_height,
            'rotation': (uniform(0, angle_max), 0, uniform(0, 2*math.pi)),
            'location': (uniform(-1, 1), uniform(-1, 1), 0)
        })

    config.write_json("ConfigFiles/" + prefix + name + ".json")
    config.write_json("ConfigFiles/default.json")
    config.clear_primitives()

def remove_blender_file_suffixes():
    for filename in glob.glob("PointClouds/*.pcd"):
        if filename.endswith("00000.pcd"):
            os.rename(filename, filename.replace("00000.pcd", ".pcd"))

def convert_pcd_files_to_csv():
    for filename in glob.glob("PointClouds/*.pcd"):
        pcd_to_csv(filename)


def main():
    print("Generating scenes...\n")

    config_name = "sample"
    samples_per_class = 3
    classes_parameters = [
        {'cylinders_count': 5, 'cylinders_radius': 0.025},
        {'cylinders_count': 5, 'cylinders_radius': 0.05},
        {'cylinders_count': 5, 'cylinders_radius': 0.1},
        {'cylinders_count': 5, 'cylinders_radius': 0.2}
    ]

    for param in classes_parameters:
        for i in range(0, samples_per_class):
            generate_cylinders_config(config_name + "_" + str(i), param['cylinders_count'], param['cylinders_radius'])
            # blensor_command = '/home/smichaud/blensor-1.0.16rc1/build/bin/blender'
            # python_script = 'blensor_script.py'
            # os.system(blensor_command + ' -b -P ' + python_script)

    # remove_blender_file_suffixes()
    # convert_pcd_files_to_csv()

    print("\nJob done !")

if __name__ == '__main__':
    main()