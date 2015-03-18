import bpy
import blensor

from scene_generation_config import SceneGenerationConfig


def add_camera(location=(-12, 0, -2), rotation=(-90, 0, 90)):
    bpy.ops.object.camera_add(location=location, rotation=rotation)

def add_point_lamp(location=(0.0, 0.0, 0.0)):
    bpy.ops.object.lamp_add(type='POINT', location=(0.0, 0.0, 0.0))

def add_primitives(primitives):
    for primitive in primitives:
        add_primitive(primitive)

def add_primitive(primitive):
    bpy.ops.mesh.primitive_cylinder_add(vertices=64,
                                        radius=0.1,
                                        depth=2.0,
                                        location=(0.0, 0.0, 0.0),
                                        rotation=(0.0, 0.0, 0.0))
    bpy.ops.mesh.primitive_cone_add(vertices=64,
                                    radius1=1.0,
                                    radius2=0.0,
                                    depth=2.0,
                                    location=(0.0, 0.0, 0.0),
                                    rotation=(0.0, 0.0, 0.0))
    bpy.ops.mesh.primitive_cube_add(location=(0.0, 0.0, 0.0),
                                    rotation=(0.0, 0.0, 0.0))
    bpy.ops.mesh.primitive_torus_add(location=(0.0, 0.0, 0.0),
                                     rotation=(0.0, 0.0, 0.0),
                                     major_radius=1.0,
                                     minor_radius=0.25,
                                     major_segments=48,
                                     minor_segments=12,
                                     use_abso=False, abso_major_rad=1.0, abso_minor_rad=0.5)

def clear_scene():
    for obj in bpy.context.scene.objects:
        bpy.context.scene.objects.unlink(obj)

def print_objects():
    for obj in bpy.context.scene.objects:
        print(obj)

def generate_pointcloud(output_pcd_file):
    bpy.data.objects['Camera.001'].local_coordinates = False
    blensor.blendodyne.scan_advanced(bpy.data.objects['Camera.001'],
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
                                     add_noisy_blender_mesh = False)
    # or
    # blensor.dispatch_scan(bpy.data.objects['Camera'], "/home/smichaud/Desktop/a.pcd")


def save_blend_file(output_blend_file):
    bpy.ops.wm.save_mainfile(filepath=output_blend_file)


def main():
    config = SceneGenerationConfig()
    config.from_json()

    clear_scene()
    add_camera()
    add_point_lamp()
    add_primitives(config.primitives)
    generate_pointcloud(config.output_pcd_file)

    save_blend_file(config.output_blend_file)

if __name__ == '__main__':
    main()