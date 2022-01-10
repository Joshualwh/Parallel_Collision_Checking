# from mesh_to_sdf import mesh_to_voxels, mesh_to_sdf
from mesh_to_sdf import get_surface_point_cloud
# from transformations.transformations import quaternion_conjugate
from urdf_parser_py.urdf import URDF

import torch
import tensorflow as tf
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
    time.sleep(3)

def test_urdf():

    chain = pk.build_serial_chain_from_urdf(open("osr_description/urdf/denso_vs060.urdf").read(), "J6")
    print(chain)
    print(chain.get_joint_parameter_names())
    
    N = 1
    th_batch = torch.rand(N, len(chain.get_joint_parameter_names()), dtype=dtype, device=d)
    chain = chain.to(dtype=dtype, device=d)

    tg_batch = chain.forward_kinematics(th_batch, end_only=False)
    homogeneous_trans_mat = torch.eye(4).repeat(7, 1, 1).type(dtype)

    i = 0
    for key in tg_batch :
        homogeneous_trans_mat_one = tg_batch[key].get_matrix()
        # print(homogeneous_trans_mat_one)
        if i == 0 :
            homogeneous_trans_mat[i] = homogeneous_trans_mat_one
        else :
            homogeneous_trans_mat[i] = homogeneous_trans_mat_one * homogeneous_trans_mat[i-1] 
        i+= 1

    print("Randomized one FK! Moving on to calculate SDF!")
    time.sleep(3)
    
    # point_start_sec = time.time()
    query_points_world_list = torch.rand((1000, 4, 1), dtype=dtype, device=d)
    query_points_world_list[:,3] = 1

    query_points_local_list = torch.matmul(homogeneous_trans_mat.repeat(len(query_points_world_list),1,1), query_points_world_list.repeat_interleave(len(homogeneous_trans_mat), dim=0))

    print(query_points_local_list)

    # for i in range(len(homogeneous_trans_mat)) : 
    #     if i == 0 :
    #         # print("I hope this only once")
    #         query_points_local = torch.matmul(homogeneous_trans_mat[i], query_points_world_list.T).T
    #         current_query_point = query_points_local
    #     else :
    #         # print("IT ran this!")
    #         current_query_point = torch.matmul(homogeneous_trans_mat[i], current_query_point.T).T
    #     # current_query_point = torch.reshape(current_query_point, (4,1))
    #     # print (current_query_point.size)
    # point_seconds = point_start_sec - time.time()

        #     current_query_point_reshaped = torch.reshape(current_query_point, (1,4))
        #     final_query_points = torch.reshape(current_query_point_reshaped[0][:3], (1,-1))

    dist = saved_cloud[i].get_sdf_in_batches(query_points_local_list, use_depth_buffer=False)
            # print(dist)
    # seconds = time.time() - start_sec
    # print (seconds)

if __name__ == "__main__":

    d = "cuda" if torch.cuda.is_available() else "cpu"
    # d = "cpu"
    dtype = torch.float64
    saved_cloud = np.empty(7, dtype=object)
    # saved_cloud = torch.empty(7, device=d)
    # print (saved_cloud)
    # saved_cloud = torch.Tensor(saved_cloud).cuda()
    precalculate_surface_point_cloud()
    test_urdf()
    # query_points(homogeneous_trans_mat)