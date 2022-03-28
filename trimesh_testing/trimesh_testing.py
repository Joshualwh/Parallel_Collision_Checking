from cv2 import transform
from mesh_to_sdf import get_surface_point_cloud
from urdf_parser_py.urdf import URDF
from trimesh.voxel import creation
from trimesh.viewer import *
from trimesh.collision import CollisionManager
from trimesh.creation import *

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
        # trimesh.points.PointCloud.show(mesh)
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

def trimesh_collision () :

    file = "osr_description/urdf/denso_vs060.urdf"
    robot = URDF.from_xml_file(file)
    links = robot.links
    n_links = len(links)
    # n_links = 3

    Collision_Checking = CollisionManager()
    for i in range(n_links):
        robot_mesh_filename = robot.links[i].collision.geometry.filename
        robot_link = robot_mesh_filename.replace('package://', '')
        # print(robot_link)
        mesh_to_insert = trimesh.load_mesh(robot_link)
        # trimesh.points.PointCloud.show(mesh_to_insert)
        print("Scanning...", i)
        name = 'link' + str(i)
        Collision_Checking.add_object(name, mesh=mesh_to_insert)
        # Collision_Checking.add_object(name, mesh=mesh_to_insert)
        # Collision_Checking.add_object(name='link_2', mesh=mesh_to_insert)
    
    names = Collision_Checking._names
    print (names)

    robot_mesh_filename = robot.links[3].collision.geometry.filename
    robot_link = robot_mesh_filename.replace('package://', '')
    mesh_to_insert = trimesh.load_mesh(robot_link)
    # trimesh.points.PointCloud.show(mesh_to_insert)

    collision_status, collision_name  = Collision_Checking.in_collision_single(mesh_to_insert,return_names=True, return_data=False)
    print (collision_status)
    # print (collision_data)
    print (collision_name)

def voxel_creation () :

    # Transformation Matrix
    voxel_transform_identity = np.identity(4)
    print(voxel_transform_identity)
    voxel_transform = voxel_transform_identity
    voxel_box = box(extents=(0.04, 0.04, 0.04), transform=voxel_transform)
    trimesh.points.PointCloud.show(voxel_box)


    # voxel_collision_manager = CollisionManager()
    # for i in range (5) :
    #     name = 'voxel_' + str(i)
    #     voxel_collision_manager.add_object(name, o1)

    # print (voxel_collision_manager._names)

def calc_sdf_fcl () :

    g1 = fcl.Box(0.04, 0.04, 0.04)
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
    # precalculate_surface_point_cloud()
    # trimesh_collision()
    voxel_creation()
    # trimesh_point_test (saved_cloud)