from mesh_to_sdf import get_surface_point_cloud
from urdf_parser_py.urdf import URDF
from trimesh.voxel import creation
from trimesh.viewer import *
from trimesh.collision import CollisionManager
from trimesh.creation import *

import pandas as pd
import torch
import trimesh
import numpy as np

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

def voxel_collision_status_1 () :

    #Create Coordinate System for Translation
    query_points_world_list = torch.zeros((64000, 4), dtype=dtype, device=dev)
    x_count = 0
    y_count = 0
    z_count = 0
    for not_sure_index in range (len(query_points_world_list)) :
        query_points_world_list[not_sure_index][0] = (0.04 * x_count) - 0.78
        query_points_world_list[not_sure_index][1] = (0.04 * y_count) - 0.78
        query_points_world_list[not_sure_index][2] = (0.04 * z_count) - 0.78
        if z_count == 40:
            y_count +=1
            z_count = 0
            if y_count == 40 :
                x_count +=1
                y_count = 0
        z_count +=1
    # print (query_points_world_list)

    # Transformation Matrix (Translation)
    voxel_collision_manager = CollisionManager()
    voxel_transform_identity = torch.zeros((4, 4), dtype=dtype, device=dev)
    for i in range (4) :
        voxel_transform_identity[i][i] = 1
    
    # Creation of 64000 Voxel Mesh, saved in voxel_collision_manager
    for count in range (len(query_points_world_list)) :
    # for count in range (64000) :
        for i in range (3) : 
            voxel_transform_identity [i][3] = query_points_world_list[count][i]
            # voxel_transform_identity [1][3] = query_points_world_list[0][1]
            # voxel_transform_identity [2][3] = query_points_world_list[0][2]
        voxel_box = box(extents=(0.04, 0.04, 0.04), transform=voxel_transform_identity)
        # trimesh.points.PointCloud.show(voxel_box)
        voxel_index = 'voxel_' + str(count)
        voxel_collision_manager.add_object(voxel_index, voxel_box)

    # names = voxel_collision_manager._names
    # print (names)

    # Import Robot Mesh to calculate minimum distance between each voxel to mesh
    file = "osr_description/urdf/denso_vs060.urdf"
    robot = URDF.from_xml_file(file)
    links = robot.links
    n_links = len(links)
    n_links = 1

    for i in range (n_links): 
        robot_mesh_filename = robot.links[i].collision.geometry.filename
        robot_link = robot_mesh_filename.replace('package://', '')
        mesh = trimesh.load_mesh(robot_link)
        print ('Distancing...' + str(i))
        collision_status, voxel_name = voxel_collision_manager.in_collision_single(mesh, transform=None, return_names=True, return_data=False)
        print(collision_status, voxel_name)

def voxel_distance_calculation () :

    #Create Coordinate System for Translation
    query_points_world_list = torch.zeros((64000, 4), dtype=dtype, device=dev)
    x_count = 0
    y_count = 0
    z_count = 0
    for not_sure_index in range (len(query_points_world_list)) :
        query_points_world_list[not_sure_index][0] = (0.04 * x_count) - 0.78
        query_points_world_list[not_sure_index][1] = (0.04 * y_count) - 0.78
        query_points_world_list[not_sure_index][2] = (0.04 * z_count) - 0.78
        if z_count == 40:
            y_count +=1
            z_count = 0
            if y_count == 40 :
                x_count +=1
                y_count = 0
        z_count +=1
    # print (query_points_world_list)

    # Import Robot Mesh to calculate minimum distance between each voxel to mesh
    file = "osr_description/urdf/denso_vs060.urdf"
    robot = URDF.from_xml_file(file)
    links = robot.links
    n_links = len(links)
    n_links = 1

    dict_per_link = {}
    df_per_link = pd.DataFrame()
    df_per_run = pd.DataFrame()
    for i in range (n_links): 
        robot_mesh_filename = robot.links[0].collision.geometry.filename
        robot_link = robot_mesh_filename.replace('package://', '')
        mesh = trimesh.load_mesh(robot_link)
        # print ('Distancing...' + str(i))
        for count in range (len(query_points_world_list)) :
            voxel_collision_manager = CollisionManager()
            voxel_transform_identity = torch.zeros((4, 4), dtype=dtype, device=dev)
            for i in range (4) :
                voxel_transform_identity[i][i] = 1
            for i in range (3) : 
                voxel_transform_identity [i][3] = query_points_world_list[count][i]
                # voxel_transform_identity [1][3] = query_points_world_list[0][1]
                # voxel_transform_identity [2][3] = query_points_world_list[0][2]
            voxel_box = box(extents=(0.04, 0.04, 0.04), transform=voxel_transform_identity)
            # trimesh.points.PointCloud.show(voxel_box)
            voxel_index = 'voxel_' + str(count)
            voxel_collision_manager.add_object(voxel_index, voxel_box)
            voxel_distance, voxel_name = voxel_collision_manager.min_distance_single(mesh, transform=None, return_name=True, return_data=False)
            # print(voxel_distance, voxel_name)
            dict_per_link[voxel_name] = voxel_distance
            df_per_link = pd.DataFrame(list(dict_per_link.items()),columns = [count,'SDF'])
        df_per_run = pd.concat((df_per_run, df_per_link), axis=1)

    return df_per_run

# Calculates the collision status between every voxel and the robot link
def voxel_collision_status () :

    #Create coordinate system for translation purposes
    query_points_world_list = torch.zeros((64000, 4), dtype=dtype, device=dev)
    x_count = 0
    y_count = 0
    z_count = 0
    for not_sure_index in range (len(query_points_world_list)) :
        query_points_world_list[not_sure_index][0] = (0.04 * x_count) - 0.78
        query_points_world_list[not_sure_index][1] = (0.04 * y_count) - 0.78
        query_points_world_list[not_sure_index][2] = (0.04 * z_count) - 0.78
        if z_count == 40:
            y_count +=1
            z_count = 0
            if y_count == 40 :
                x_count +=1
                y_count = 0
        z_count +=1

    # Import all the robot arm links
    file = "osr_description/urdf/denso_vs060.urdf"
    robot = URDF.from_xml_file(file)
    links = robot.links
    n_links = len(links)

    # Dictionary is created to store the data
    dict_per_link = {}

    # Pandas DataFrame is used to store the data to output as .csv files
    df_per_link = pd.DataFrame()
    df_per_run = pd.DataFrame()

    # The main process of the precomputation 
    for i in range (n_links): 
        robot_mesh_filename = robot.links[i].collision.geometry.filename
        robot_link = robot_mesh_filename.replace('package://', '')
        mesh = trimesh.load_mesh(robot_link)
        for count in range (len(query_points_world_list)) :
            voxel_collision_manager = CollisionManager()
            voxel_transform_identity = torch.zeros((4, 4), dtype=dtype, device=dev)
            for identity_index in range (4) :
                voxel_transform_identity[identity_index][identity_index] = 1
            for i in range (3) : 
                voxel_transform_identity [i][3] = query_points_world_list[count][i]
            voxel_box = box(extents=(0.04, 0.04, 0.04), transform=voxel_transform_identity)
            voxel_index = 'voxel_' + str(count)
            voxel_collision_manager.add_object(voxel_index, voxel_box)
            voxel_collision = voxel_collision_manager.in_collision_single(mesh, transform=None, return_names=False, return_data=False)
            # print(voxel_collision, voxel_name)
            dict_per_link[voxel_index] = voxel_collision
            df_per_link = pd.DataFrame(list(dict_per_link.items()),columns = [voxel_index,'SDF'])
        df_per_run = pd.concat((df_per_run, df_per_link), axis=1)

    return df_per_run

if __name__ == "__main__":
    dev = "cpu"
    dtype = torch.float32
    # trimesh_collision()
    # # voxel_collision_status()
    # trimesh_collision_status = pd.DataFrame()
    # trimesh_collision_status = voxel_collision_status()
    # trimesh_collision_status.to_csv('trimesh_collision_status.csv')
    # voxel_collision_status_1()
    # voxel_distance_calculation()
    # trimesh_sdf = pd.DataFrame()
    # trimesh_sdf = voxel_distance_calculation()
    # trimesh_sdf.to_csv('trimesh_sdf.csv')
    # trimesh_point_test (saved_cloud)