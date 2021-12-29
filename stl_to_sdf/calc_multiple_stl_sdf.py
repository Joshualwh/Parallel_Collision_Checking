from mesh_to_sdf import mesh_to_voxels, mesh_to_sdf
from mesh_to_sdf import get_surface_point_cloud, sample_sdf_near_surface, scale_to_unit_cube
from urdf_parser_py.urdf import URDF

import trimesh
import skimage
import pyrender
import numpy as np
import os
import time

saved_cloud = np.empty(7, dtype=object)

def precalculate_surface_point_cloud() :

    file = "osr_description/urdf/denso_vs060.urdf"
    robot = URDF.from_xml_file(file)
    links = robot.links
    n_links = len(links)
    for i in range(n_links):
        robot_mesh_filename = robot.links[i].collision.geometry.filename
        robot_link = robot_mesh_filename.replace('package://', '')
        # print(robot_link)
        mesh = trimesh.load_mesh(robot_link)
        print("Scanning...", i)
        cloud = get_surface_point_cloud(mesh, surface_point_method='scan', scan_count=20, scan_resolution=400)

        saved_cloud[i] = cloud

def query_points() :

    query_points = np.array([[2,1,1]])
    for i in range(len(saved_cloud)):
        seconds = time.time()
        # point = mesh_to_sdf(mesh, query_points)
        dist = saved_cloud[i].get_sdf_in_batches(query_points, use_depth_buffer=False)

        print(dist)
        print(time.time() - seconds)

if __name__ == "__main__":
    precalculate_surface_point_cloud()
    query_points()