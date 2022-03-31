from urdf_parser_py.urdf import URDF
# from trimesh.voxel import creation
# from trimesh.viewer import *
# from trimesh.collision import CollisionManager
# from trimesh.creation import *

import pandas as pd
import torch
import numpy as np
import pytorch_kinematics as pk
import time
import voxel_conversion

# Read excel file, create dictionary
# Create 7 dict for each link
def read_dict () :

    dict_from_csv = pd.read_csv('trimesh_collision_status.csv', header=None, index_col=0, squeeze=True).to_dict()
    voxel_index = list(dict_from_csv[1].keys())
    voxel_index.pop(0)
    voxel_index = [int(x) for x in voxel_index]
    # voxel_index_str = [str(x) for x in voxel_index]
    # print(voxel_index[0])
    # print(dict_from_csv[1])
    # print(dict_from_csv[2][voxel_index[1234]])

    for i in range (len(voxel_index)) :
        # link_0_dict[voxel_index[i]] = dict_from_csv[2]
        link_0_dict[voxel_index[i]] = dict_from_csv[2][voxel_index[i]]
        link_1_dict[voxel_index[i]] = dict_from_csv[4][voxel_index[i]]
        link_2_dict[voxel_index[i]] = dict_from_csv[6][voxel_index[i]]
        link_3_dict[voxel_index[i]] = dict_from_csv[8][voxel_index[i]]
        link_4_dict[voxel_index[i]] = dict_from_csv[10][voxel_index[i]]
        link_5_dict[voxel_index[i]] = dict_from_csv[12][voxel_index[i]]
        link_6_dict[voxel_index[i]] = dict_from_csv[14][voxel_index[i]]

    # print(link_0_dict['23222'])
    print('Finished creating dict for each link...')

# Pytorch_Kinematics to get homogeneous transformation matrix
def pytorch_kinematics_implementation () :
    chain = pk.build_serial_chain_from_urdf(open("osr_description/urdf/denso_vs060.urdf").read(), "J6")
    print(chain)
    # print(chain.get_joint_parameter_names())
    
    N = 1
    th_batch = torch.rand(N, len(chain.get_joint_parameter_names()), dtype=dtype, device=dev)
    chain = chain.to(dtype=dtype, device=dev)

    tg_batch = chain.forward_kinematics(th_batch, end_only=False)
    homogeneous_trans_mat = torch.eye(4, dtype=dtype, device=dev).repeat(7, 1, 1)

    link_0_trans_mat = torch.eye(4, dtype=dtype, device=dev)
    # print(link_0_trans_mat)

    homogeneous_trans_mat[0] = link_0_trans_mat
    i = 1
    for key in tg_batch :
        homogeneous_trans_mat_one = tg_batch[key].get_matrix()
        # print(homogeneous_trans_mat_one)
        if i == 1 :
            homogeneous_trans_mat[i] = homogeneous_trans_mat_one
        else :
            homogeneous_trans_mat[i] = homogeneous_trans_mat_one * homogeneous_trans_mat[i-1] 
        i+= 1

    print("Randomized one FK and moving on...")
    # time.sleep(1)
    return homogeneous_trans_mat


# Input of randomized points, which will be converted to voxels, search dictionary for collision status
def main_calculation () :
    # Still waiting for implementation...
    query_points_world_list = torch.rand((1000, 4, 1), dtype=dtype, device=dev)
    query_points_world_list[:,3] = 1

    query_points_local_list = torch.matmul(homogeneous_trans_mat.repeat(len(query_points_world_list),1,1), query_points_world_list.repeat_interleave(len(homogeneous_trans_mat), dim=0))
    # print(query_points_local_list)
    query_points_local_list_reshaped = torch.squeeze(query_points_local_list)
    # print(query_points_local_list_reshaped)
    ids = torch.tensor([3], device=dev).repeat(7000)

    mask = torch.ones_like(query_points_local_list_reshaped).scatter_(1, ids.unsqueeze(1), 0.)
    query_points_local_list_final = query_points_local_list[mask.bool()].view(7000, 3)
    # print(query_points_local_list_final)

    gridcenter = torch.zeros(3, dtype=torch.float32, device=dev)
    gridextent = torch.ones(3, dtype=torch.float32) *0.8
    numvoxels = torch.ones(3, dtype=torch.int16) *40

    # Coordinate to voxel conversion
    vox_con = voxel_conversion.VoxelConversion(gridcenter, gridextent, numvoxels)
    
    # robot = URDF.from_xml_file(file)
    # links = robot.links
    n_links = 7
    
    # start_seconds = time.time()
    for links in range(n_links) :  
        # print(links)
        # start_seconds_3 = time.time()
        for i in range (len(query_points_local_list_final)) :
            # start_seconds_1 = time.time()
            vox_ind = vox_con.toIndex(query_points_local_list_final[i])
            # end_seconds_1 = time.time() - start_seconds_1
            # print(end_seconds_1)
            # print (vox_ind)
            # start_seconds_2 = time.time()
            if vox_ind != -1 : 
                # Put voxel found into dict to find collision status
                collision_status = link_0_dict[vox_ind]
                # print(collision_status)
                    
                # if collision_status == 'True' :
                #     print ('In Collision!')
                #     break
                # else :
                #     print ('All Cool!')
            # end_seconds_2 = time.time() - start_seconds_2
            # print(end_seconds_2)
        # end_seconds_3 = time.time() - start_seconds_3
        # print(end_seconds_3)
    # time_taken = time.time() - start_seconds
    # print(time_taken)


if __name__ == "__main__":
    dev = "cpu"
    # dev = "cuda" if torch.cuda.is_available() else "cpu"
    dtype = torch.float32
    link_0_dict = {}
    link_1_dict = {}
    link_2_dict = {}
    link_3_dict = {}
    link_4_dict = {}
    link_5_dict = {}
    link_6_dict = {}
    read_dict()
    homogeneous_trans_mat = pytorch_kinematics_implementation()
    collision_status = main_calculation()
    # print(collision_status)
    # print(homogeneous_trans_mat)