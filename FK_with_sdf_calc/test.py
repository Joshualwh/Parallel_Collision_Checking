# from mesh_to_sdf import mesh_to_voxels, mesh_to_sdf
from mesh_to_sdf import get_surface_point_cloud
# from transformations.transformations import quaternion_conjugate
from urdf_parser_py.urdf import URDF

import torch
import pytorch_kinematics as pk
import trimesh
import numpy as np
import time

saved_cloud = np.empty(7, dtype=object)
homogeneous_trans_mat = np.empty(6, dtype=object)

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

# def quat_pos_from_transform3d(tg):
#     m = tg.get_matrix()
#     pos = m[:, :3, 3]
#     rot = pk.matrix_to_quaternion(m[:, :3, :3])
#     return pos, rot

# def quaternion_equality(a, b):
#     # negative of a quaternion is the same rotation
#     return torch.allclose(a, b) or torch.allclose(a, -b)

def test_urdf():
    chain = pk.build_serial_chain_from_urdf(open("osr_description/urdf/denso_vs060.urdf").read(), "J6")
    print(chain)
    print(chain.get_joint_parameter_names())
    # th = [0.0, -math.pi / 4.0, 0.0, math.pi / 2.0, 0.0, math.pi / 4.0, 0.0]
    # ret = chain.forward_kinematics(th, end_only=False)
    # tg = ret['J6']
    # print(tg)
    # I have no idea what these 3 lines are for? 
    # pos, rot = quat_pos_from_transform3d(tg)
    # assert quaternion_equality(rot, torch.tensor([7.07106781e-01, 0, -7.07106781e-01, 0]))
    # assert torch.allclose(pos, torch.tensor([-6.60827561e-01, 0, 3.74142136e-01]))

    N = 1
    d = "cuda" if torch.cuda.is_available() else "cpu"
    dtype = torch.float64

    th_batch = torch.rand(N, len(chain.get_joint_parameter_names()), dtype=dtype, device=d)
    chain = chain.to(dtype=dtype, device=d)

    tg_batch = chain.forward_kinematics(th_batch, end_only=False)
    # print(tg_batch)
    i = 0
    for key in tg_batch :
        homogeneous_trans_mat_one = tg_batch[key].get_matrix()
        print(homogeneous_trans_mat_one)

        homogeneous_trans_mat[i] = homogeneous_trans_mat_one
        i+= 1

    # elapsed = time.time() - start
    # print(m)
    # print("elapsed {}s for N={} when parallel".format(elapsed, N))

    print("Randomized one FK! Moving on to calculate SDF!")
    time.sleep(3)

    # start = time.time()
    # elapsed = 0
    # for i in range(N):
    #     tg = chain.forward_kinematics(th_batch[i])
    #     elapsed += time.time() - start
    #     start = time.time()
    #     assert torch.allclose(tg.get_matrix().view(4, 4), m[i])
    # print("elapsed {}s for N={} when serial".format(elapsed, N))

def query_points(homogeneous_trans_mat) :

    query_points_world = np.array([[1],
                                   [1],
                                   [1],
                                   [1]])

    # print(homogeneous_trans_mat)

    for i in range(len(homogeneous_trans_mat)) : 
        if i == 0 :
            # print("I hope this only once")
            query_points_local = np.dot(homogeneous_trans_mat[i], query_points_world)
            current_query_point = query_points_local
        else :
            # print("IT ran this!")
            current_query_point = np.dot(homogeneous_trans_mat[i], current_query_point)
        current_query_point = current_query_point.reshape(4,1)
        print (current_query_point)

        current_query_point_reshaped = current_query_point.reshape(1,4)
        final_query_points = current_query_point_reshaped[0][:3].reshape(1,-1)
        dist = saved_cloud[i].get_sdf_in_batches(final_query_points, use_depth_buffer=False)
        print(dist)    

    # for i in range(len(homogeneous_trans_mat)) :  
    #     query_points_local = np.dot(homogeneous_trans_mat[i], query_points_world)
    #     # print (query_points_local)
    #     query_points_local_reshaped = np.reshape(query_points_local, (1,4))
    #     final_query_points = query_points_local_reshaped[0][:3].reshape(1,-1))
    #     dist = saved_cloud[i].get_sdf_in_batches(final_query_points, use_depth_buffer=False)
    #     print(dist)

if __name__ == "__main__":
    precalculate_surface_point_cloud()
    test_urdf()
    query_points(homogeneous_trans_mat)