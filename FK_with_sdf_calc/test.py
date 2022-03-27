# from cgi import test
# import os
# from unicodedata import decimal
# os.environ['PYOPENGL_PLATFORM'] = 'osmesa'

import pandas as pd
import torch
# import tensorflow as tf
import pytorch_kinematics as pk
import trimesh
import numpy as np
import time

from mesh_to_sdf import get_surface_point_cloud
from urdf_parser_py.urdf import URDF

from voxel_conversion import VoxelConversion

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

def test_urdf(dev, run_time):

    chain = pk.build_serial_chain_from_urdf(open("osr_description/urdf/denso_vs060.urdf").read(), "J6")
    print(chain)
    print(chain.get_joint_parameter_names())
    
    N = 1
    th_batch = torch.rand(N, len(chain.get_joint_parameter_names()), dtype=dtype, device=dev)
    chain = chain.to(dtype=dtype, device=dev)

    tg_batch = chain.forward_kinematics(th_batch, end_only=False)
    homogeneous_trans_mat = torch.eye(4, dtype=dtype, device=dev).repeat(7, 1, 1)

    i = 0
    for key in tg_batch :
        homogeneous_trans_mat_one = tg_batch[key].get_matrix()
        if i == 0 :
            homogeneous_trans_mat[i] = homogeneous_trans_mat_one
        else :
            homogeneous_trans_mat[i] = homogeneous_trans_mat_one * homogeneous_trans_mat[i-1] 
        i+= 1

    print("Randomized one FK! Moving on to calculate SDF!")
    time.sleep(1)
    
    # dfObj_per_run = pd.DataFrame()
    # dfObj_per_run['Timing'] = ['0']
    point_start_seconds = time.time()
    query_points_world_list = torch.zeros((64000, 4, 1), dtype=dtype, device=dev)
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
        # print(x_count, y_count, z_count)
        z_count +=1

    query_points_world_list[:,3] = 1
    # print (query_points_world_list)

    # query_points_local_list = torch.matmul(homogeneous_trans_mat.repeat(len(query_points_world_list),1,1), query_points_world_list.repeat_interleave(len(homogeneous_trans_mat), dim=0))
    # print(query_points_local_list)
    # query_points_local_list_reshaped = torch.squeeze(query_points_local_list)
    query_points_local_list_reshaped = torch.squeeze(query_points_world_list)
    print(query_points_local_list_reshaped.size)
    ids = torch.tensor([3], device=dev).repeat(64000)

    mask = torch.ones_like(query_points_local_list_reshaped).scatter_(1, ids.unsqueeze(1), 0.)
    query_points_local_list_final = query_points_world_list[mask.bool()].view(64000, 3)
    # query_points_local_list_final = torch.round(query_points_local_list_final * 10 ** 2) / (10 ** 2)
    print(query_points_local_list_final)
    point_seconds = time.time() - point_start_seconds
    print (point_seconds)
    # dfObj_per_run[run_time] = [point_seconds]

    testing_dict_per_link = {}
    testing_df_per_link = pd.DataFrame()
    testing_dict_per_run = pd.DataFrame()
    dist_start_seconds = time.time()
    for i in range(len(chain.get_joint_parameter_names())+1):
        print(saved_cloud[i])
        dist = saved_cloud[i].get_sdf_in_batches(query_points_local_list_final,run_time, use_depth_buffer=False)
        # dist, dfObj_per_link = saved_cloud[i].get_sdf_in_batches(query_points_local_list_final, run_time, use_depth_buffer=False)
        # print(len(dist[0]))
        # print(dist)
        # print(dist[0])

        for index in range (len(query_points_local_list_final)) : 
            # testing_dict_per_link[i*64000:((i+1)*64000)-1] = dist[0][index]
            # For Mac Laptop
            # testing_dict_per_link[index] = dist[index]
            # For Desktop
            testing_dict_per_link[index] = dist[0][index] 
        voxel_index_number = 'vox_ind_' + str(i)
        print(voxel_index_number)
        testing_df_per_link = pd.DataFrame(list(testing_dict_per_link.items()),columns = [voxel_index_number,'SDF'])

        # print (testing_dict_per_run)
        # # testing_dict_per_run = testing_dict_per_run.append(testing_dict_per_link, ignore_index=True)
        testing_dict_per_run = pd.concat((testing_dict_per_run, testing_df_per_link), axis=1)
    # print (dfObj_per_run)
    
    dist_seconds = time.time() - dist_start_seconds
    print ("Total timing: ", dist_seconds)

    return testing_dict_per_run
    # return dfObj_per_run
    # print(testing_dict)

if __name__ == "__main__":

    # dev = "cuda" if torch.cuda.is_available() else "cpu"
    dev = "cpu"
    # dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    dtype = torch.float32
    saved_cloud = np.empty(7, dtype=object)
    precalculate_surface_point_cloud()
    
    run_time = 1
    # dfObj_total = pd.DataFrame()
    testing_dict = pd.DataFrame()
    # test_urdf(dev, run_time)
    # for run_time in range (1) :
    testing_dict = test_urdf(dev, run_time)
    testing_dict.to_csv('testing_dict_2.csv')
        # dfObj_total = pd.concat([dfObj_total, dfObj_per_run], axis=1)

    # dfObj_total.to_csv('timing_results_02.csv')

    # with open('testing_dict.csv', 'w') as f:
    #     for key in testing_dict.keys():
    #         f.write("%s,%s\n"%(key,testing_dict[key]))

    # voxel_conversion.VoxelConversion()


    # from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())

    # for run_time in range (1) :
    #     dfObj_per_run = test_urdf(dev, run_time)
    #     run_time+=1
    #     dfObj_total = pd.concat([dfObj_total, dfObj_per_run], axis=1)

    # dfObj_total.to_csv('timing_results_02.csv')

    # from torch.profiler import profile, record_function, ProfilerActivity

    # with profile(activities=[ProfilerActivity.CPU,], record_shapes=True) as prof:
    #     test_urdf(dev)
