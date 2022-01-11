from mesh_to_sdf import get_surface_point_cloud
from urdf_parser_py.urdf import URDF

import torch
import pytorch_kinematics as pk
import trimesh
import numpy as np
import time

# Global variables
saved_cloud = np.empty(7, dtype=object)
homogeneous_trans_mat = np.empty(6, dtype=object)

#Calculates the surface point cloud for each link
def precalculate_surface_point_cloud() :

    #Load the urdf file which opens the meshes to scan
    file = "osr_description/urdf/denso_vs060.urdf"
    robot = URDF.from_xml_file(file)
    links = robot.links
    n_links = len(links)
    #Scans each link for their point cloud and saves it for later computation of sdf
    for i in range(n_links):
        robot_mesh_filename = robot.links[i].collision.geometry.filename
        robot_link = robot_mesh_filename.replace('package://', '')
        mesh = trimesh.load_mesh(robot_link)
        print("Scanning...", i)
        cloud = get_surface_point_cloud(mesh, surface_point_method='scan', scan_count=20, scan_resolution=400)
        saved_cloud[i] = cloud

    print("Scanning Done! Moving on to FK!")
    time.sleep(3)

#Current testing forward kinematics for ONCE only
def test_urdf():
    chain = pk.build_serial_chain_from_urdf(open("osr_description/urdf/denso_vs060.urdf").read(), "J6")
    print(chain)
    print(chain.get_joint_parameter_names())

    N = 1
    d = "cuda" if torch.cuda.is_available() else "cpu"
    dtype = torch.float64

    th_batch = torch.rand(N, len(chain.get_joint_parameter_names()), dtype=dtype, device=d)
    chain = chain.to(dtype=dtype, device=d)

    tg_batch = chain.forward_kinematics(th_batch, end_only=False)
    i = 0
    for key in tg_batch :
        homogeneous_trans_mat_one = tg_batch[key].get_matrix()
        print(homogeneous_trans_mat_one)

        homogeneous_trans_mat[i] = homogeneous_trans_mat_one
        i+= 1

    print("Randomized one FK! Moving on to calculate SDF!")
    time.sleep(3)

#Loads in the transformation matrix for computation 
def query_points(homogeneous_trans_mat) :

    #Now define a random point (world coordinate) as the query point
    query_points_world = np.array([[1],
                                   [1],
                                   [1],
                                   [1]])

    for i in range(len(homogeneous_trans_mat)) :
        #Transformation of world coordinate to local coordinates 
        if i == 0 :
            query_points_local = np.dot(homogeneous_trans_mat[i], query_points_world)
            current_query_point = query_points_local
        else :
            current_query_point = np.dot(homogeneous_trans_mat[i], current_query_point)
        current_query_point = current_query_point.reshape(4,1)
        print (current_query_point)

        current_query_point_reshaped = current_query_point.reshape(1,4)
        final_query_points = current_query_point_reshaped[0][:3].reshape(1,-1)
        print(final_query_points)
        #Calculation of sdf
        start_sec = time.time()
        dist = saved_cloud[i].get_sdf_in_batches(final_query_points, use_depth_buffer=False)
        seconds = time.time() - start_sec
        # print (dist)
        print (seconds)

if __name__ == "__main__":
    precalculate_surface_point_cloud()
    test_urdf()
    query_points(homogeneous_trans_mat)