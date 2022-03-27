from mesh_to_sdf import get_surface_point_cloud
from urdf_parser_py.urdf import URDF
from trimesh.voxel import creation
from trimesh.viewer import *

import pandas as pd
import torch
# import tensorflow as tf
import pytorch_kinematics as pk
import trimesh
import numpy as np
import time
import fcl


def precalculate_surface_point_cloud() :

    file = "osr_description/urdf/denso_vs060.urdf"
    robot = URDF.from_xml_file(file)
    links = robot.links
    # n_links = len(links)
    n_links = 1

    for i in range(n_links):
        robot_mesh_filename = robot.links[i].collision.geometry.filename
        robot_link = robot_mesh_filename.replace('package://', '')
        # print(robot_link)
        mesh = trimesh.load_mesh(robot_link)
        print("Scanning...", i)
        trimesh.points.PointCloud.show(mesh)
        # wanted_mesh = trimesh.Scene(mesh)
        # centroid = wanted_mesh.centroid
        point = (0,0,0)
        pitch = 0.04
        radius = 20
        voxelized = creation.local_voxelize(mesh, point, pitch, radius, fill=True)
        # voxelized_points = voxelized.points
        # print(voxelized_points)
        voxelized.show()
        # trimesh.voxel.creation.local_voxelize(mesh, point, pitch, radius, fill=True, **kwargs)
        cloud = get_surface_point_cloud(mesh, surface_point_method='scan', scan_count=20, scan_resolution=400)

        # cloud.show()
        # print (centroid)

        saved_cloud[i] = cloud


    print("Scanning Done! Moving on to FK!")
    time.sleep(1)

def calc_sdf_fcl () :

    g1 = fcl.Box(1,2,3)
    t1 = fcl.Transform()
    o1 = fcl.CollisionObject(g1, t1)

    g2 = fcl.Cone(1,3)
    t2 = fcl.Transform()
    o2 = fcl.CollisionObject(g2, t2)

    request = fcl.DistanceRequest()
    result = fcl.DistanceResult()

    ret = fcl.distance(o1, o2, request, result)

# After calling fcl.distance(), ret contains the minimum distance between the two objects and result contains information about the closest points on the objects. If ret is negative, the objects are in collision. For more information about available parameters for distance requests and results, see fcl/collision_data.py.


# def trimesh_point_test (saved_point_cloud) :

#     for i in range(len(saved_point_cloud)):
#         trimesh.points.PointCloud.show(saved_point_cloud[i])

# def calc() :
#     mesh = trimesh.load_mesh('urdf/meshes/BASE.stl')
#     mesh = scale_to_unit_cube(mesh)

#     print("Scanning...")
#     cloud = get_surface_point_cloud(mesh, surface_point_method='scan', scan_count=20, scan_resolution=400)

#     cloud.show()

if __name__ == "__main__":
    dev = "cpu"
    dtype = torch.float32
    saved_cloud = np.empty(7, dtype=object)
    precalculate_surface_point_cloud()
    # trimesh_point_test (saved_cloud)