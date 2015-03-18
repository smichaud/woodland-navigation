import bpy
import blensor
import os

def createScene():
    # Area fo interest: ([-1, 1], [-1, 1], [-1, 1])
    bpy.ops.object.camera_add(location=(-12, 0, -2), rotation=(-90, 0, 90))
    # bpy.ops.object.lamp_add(type='POINT', radius=1.0, location=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0))
    bpy.ops.mesh.primitive_cylinder_add(vertices=64, radius=0.1, depth=2.0, end_fill_type='NGON', view_align=False,
                                        enter_editmode=False, location=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0))

def clearScene():
  for obj in bpy.context.scene.objects:
        bpy.context.scene.objects.unlink(obj)

def printObjects():
    for obj in bpy.context.scene.objects:
        print(obj)

def generatePointCloud(outputPCDFile):
    bpy.data.objects["Camera.001"].local_coordinates = False
    blensor.blendodyne.scan_advanced(bpy.data.objects["Camera.001"],
                                     rotation_speed = 10.0,
                                     simulation_fps=24,
                                     angle_resolution = 0.1728,
                                     max_distance = 120,
                                     evd_file= outputPCDFile,
                                     noise_mu=0.0, noise_sigma=0.03,
                                     start_angle = 0.0,
                                     end_angle = 360.0,
                                     evd_last_scan=True,
                                     add_blender_mesh = False,
                                     add_noisy_blender_mesh = False)
    # or
    # blensor.dispatch_scan(bpy.data.objects["Camera"], "/home/smichaud/Desktop/a.pcd")


def saveBlendFile(outputBlendFile):
    bpy.ops.wm.save_mainfile(filepath=outputBlendFile)


def main():
    workingRepository = os.getcwd() + '/'
    outputPCDFile = 'PointClouds/output.pcd'
    outputBlendFile = 'BlendFiles/GeneratedScene.blend'

    printObjects()

    clearScene()
    createScene()
    generatePointCloud(workingRepository + outputPCDFile)

    printObjects()

    saveBlendFile(workingRepository + outputBlendFile)

if __name__ == "__main__":
    main()