from mesh_to_sdf import get_surface_point_cloud
from urdf_parser_py.urdf import URDF

import pandas as pd
import torch
# import tensorflow as tf
import pytorch_kinematics as pk
import trimesh
import numpy as np
import time

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


    print("Scanning Done! Moving on to FK!")
    time.sleep(1)

if __name__ == "__main__":
    dev = "cpu"
    dtype = torch.float32
    saved_cloud = np.empty(7, dtype=object)
    precalculate_surface_point_cloud()