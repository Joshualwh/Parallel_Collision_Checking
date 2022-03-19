from mesh_to_sdf import get_surface_point_cloud
from urdf_parser_py.urdf import URDF

import torch
import pytorch_kinematics as pk
import trimesh
import numpy as np
import time
import pandas as pd

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
    # d = "cuda" if torch.cuda.is_available() else "cpu"
    d= "cpu"
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

    # dfObj_per_run = pd.DataFrame()
    # dfObj_per_run['Timing'] = ['0']
    point_start_seconds = time.time()
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
        # print(final_query_points)
        point_seconds = time.time() - point_start_seconds
        print (point_seconds)
        # dfObj_per_run[run_time] = [point_seconds]
        #Calculation of sdf
        dist_start_seconds = time.time()
        dist = saved_cloud[i].get_sdf_in_batches(final_query_points, run_time, use_depth_buffer=False)
        # dist, dfObj_per_link = saved_cloud[i].get_sdf_in_batches(final_query_points, run_time, use_depth_buffer=False)
        # dfObj_per_run = dfObj_per_run.append(dfObj_per_link, ignore_index=True)
        seconds = time.time() - dist_start_seconds
        # print (dist)
        print (seconds)

        # return dfObj_per_run

if __name__ == "__main__":
    # Global variables
    saved_cloud = np.empty(7, dtype=object)
    homogeneous_trans_mat = np.empty(6, dtype=object)
    precalculate_surface_point_cloud()
    test_urdf()
    run_time = 1
    # dfObj_total = pd.DataFrame()

    # for run_time in range (1) :
    #     dfObj_per_run = query_points(homogeneous_trans_mat, run_time)
    #     run_time+=1
    #     dfObj_total = pd.concat([dfObj_total, dfObj_per_run], axis=1)
    
    # print (dfObj_total)

    # dfObj_total.to_csv('timing_results_01.csv')
    query_points(homogeneous_trans_mat)