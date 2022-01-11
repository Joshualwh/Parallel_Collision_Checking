import os
os.environ['PYOPENGL_PLATFORM'] = 'osmesa'

from mesh_to_sdf import get_surface_point_cloud
from urdf_parser_py.urdf import URDF

import torch
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

def test_urdf(dev):

    chain = pk.build_serial_chain_from_urdf(open("osr_description/urdf/denso_vs060.urdf").read(), "J6")
    print(chain)
    print(chain.get_joint_parameter_names())
    
    N = 1
    th_batch = torch.rand(N, len(chain.get_joint_parameter_names()), dtype=dtype, device=dev)
    chain = chain.to(dtype=dtype, device=dev)

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
    
    point_start_seconds = time.time()
    query_points_world_list = torch.rand((1000, 4, 1), dtype=dtype, device=dev)
    query_points_world_list[:,3] = 1

    query_points_local_list = torch.matmul(homogeneous_trans_mat.repeat(len(query_points_world_list),1,1), query_points_world_list.repeat_interleave(len(homogeneous_trans_mat), dim=0))
    # print(query_points_local_list)
    query_points_local_list_reshaped = torch.squeeze(query_points_local_list)
    # print(query_points_local_list_reshaped)
    ids = torch.tensor([3]).repeat(7000)

    mask = torch.ones_like(query_points_local_list_reshaped).scatter_(1, ids.unsqueeze(1), 0.)
    query_points_local_list_final = query_points_local_list[mask.bool()].view(7000, 3)
    # print(query_points_local_list_final)
    point_seconds = time.time() - point_start_seconds
    print (point_seconds)

    dist_start_seconds = time.time()
    for i in range(len(chain.get_joint_parameter_names())):
        dist = saved_cloud[i].get_sdf_in_batches(query_points_local_list_final, use_depth_buffer=False)
    # print(dist)
    dist_seconds = time.time() - dist_start_seconds
    print (dist_seconds)

if __name__ == "__main__":

    # d = "cuda" if torch.cuda.is_available() else "cpu"
    # dev = torch.device("cpu")
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    dtype = torch.float64
    saved_cloud = np.empty(7, dtype=object)
    precalculate_surface_point_cloud()
    from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())
    from torch.profiler import profile, record_function, ProfilerActivity

    with profile(activities=[ProfilerActivity.CPU,], record_shapes=True) as prof:
        test_urdf(dev)