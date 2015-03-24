import bpy
import blensor
import math
from mathutils import Matrix

from scene_generation_config import SceneGenerationConfig

# Default values for a ([-1, 1], [-1, 1], [-1, 1]) area of interest
def move_camera(location=(-5.2, 0, 1), rotation=(math.radians(90), 0, math.radians(-90))):
    camera_id = get_camera_id()
    bpy.data.objects[camera_id].location = (-5.2, 0, 1)
    bpy.data.objects[camera_id].rotation_euler=(math.radians(90), 0, math.radians(-90))

def add_point_lamp(location=(0.0, 0.0, 0.0)):
    bpy.ops.object.lamp_add(type='POINT', location=(0.0, 0.0, 0.0))

def add_primitives(primitives):
    for primitive in primitives:
        add_primitive(primitive)

def add_primitive(primitive):
    location = tuple(primitive['location']) if ('location' in primitive) else (0,0,0)
    rotation = tuple(primitive['rotation']) if ('rotation' in primitive) else (0,0,0)
    radius = primitive['radius'] if ('radius' in primitive) else 0
    radius1 = primitive['radius1'] if ('radius1' in primitive) else 0
    radius2 = primitive['radius2'] if ('radius2' in primitive) else 0
    depth = primitive['depth'] if ('depth' in primitive) else 0

    if 'type' in primitive:
        if primitive['type'] == 'cylinder':
            bpy.ops.mesh.primitive_cylinder_add(vertices=64,
                                                radius=radius,
                                                depth=depth,
                                                location=location,
                                                rotation=rotation)
        elif primitive['type'] == 'sphere':
            bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=4,
                                                  location=location,
                                                  rotation=rotation,
                                                  size=radius)
        elif primitive['type'] == 'cube':
            bpy.ops.mesh.primitive_cube_add(location=location,
                                            rotation=rotation)
        elif primitive['type'] == 'cone':
            bpy.ops.mesh.primitive_cone_add(vertices=64,
                                            radius1=radius1,
                                            radius2=radius2,
                                            depth=depth,
                                            location=location,
                                            rotation=rotation)
        elif primitive['type'] == 'torus':
            bpy.ops.mesh.primitive_torus_add(location=location,
                                             rotation=rotation,
                                             minor_radius=radius1,
                                             major_radius=radius2,
                                             major_segments=48,
                                             minor_segments=12)

def clear_scene_meshes():
    for obj in bpy.context.scene.objects:
        if obj.type == 'MESH':
            bpy.context.scene.objects.unlink(obj)

def print_objects():
    for obj in bpy.context.scene.objects:
        print(obj)

def get_camera_id():
    camera_id = ""
    for obj in bpy.context.scene.objects:
        if obj.type == 'CAMERA' and camera_id == "":
            camera_id = obj.name
        elif obj.type == 'CAMERA' and camera_id != None:
            print("** WARNING ** Multiple camera found !")

    return camera_id

def generate_pointcloud(output_pcd_file):
    camera_id = get_camera_id()
    blensor.blendodyne.scan_advanced(bpy.data.objects[camera_id],
                                     rotation_speed = 10.0,
                                     simulation_fps=24,
                                     angle_resolution = 0.1728,
                                     max_distance = 120,
                                     evd_file= output_pcd_file,
                                     noise_mu=0.0, noise_sigma=0.03,
                                     start_angle = 0.0,
                                     end_angle = 360.0,
                                     evd_last_scan=True,
                                     add_blender_mesh = False,
                                     add_noisy_blender_mesh = False,
                                     world_transformation = bpy.data.objects[camera_id].matrix_world)
    # or
    # blensor.dispatch_scan(bpy.data.objects['Camera'], "/home/smichaud/Desktop/a.pcd")


def save_blend_file(output_blend_file):
    bpy.ops.wm.save_mainfile(filepath=output_blend_file)


def main():
    config = SceneGenerationConfig()
    config.read_json()

    clear_scene_meshes()
    move_camera()
    add_primitives(config.primitives)

    generate_pointcloud(config.output_pcd_file)

    save_blend_file(config.output_blend_file)

if __name__ == '__main__':
    main()