from mesh_to_sdf import get_surface_point_cloud
from urdf_parser_py.urdf import URDF
from trimesh.voxel import creation
from trimesh.viewer import *
from trimesh.collision import CollisionManager
from trimesh.creation import *

import pandas as pd
import torch
import numpy as np
import pytorch_kinematics as pk
import time

# Read excel file, create dictionary
def read_dict () :

    dict_from_csv = pd.read_csv('trimesh_collision_status.csv', header=None, index_col=1, squeeze=True).to_dict()
    # print(dict_from_csv.keys())
    # print(dict_from_csv[1])
    # print(dict_from_csv[0]['voxel_23220'])
    print(dict_from_csv[2]['voxel_23220'])
    # print(dict_from_csv[3]['voxel_23220'])
    print(dict_from_csv[4]['voxel_23220'])
    # print(dict_from_csv[5]['voxel_23220'])
    print(dict_from_csv[6]['voxel_23220'])
    # print(dict_from_csv[7]['voxel_23220'])
    print(dict_from_csv[8]['voxel_23220'])
    # print(dict_from_csv[9]['voxel_23220'])
    print(dict_from_csv[10]['voxel_23220'])
    # print(dict_from_csv[11]['voxel_23220'])
    print(dict_from_csv[12]['voxel_23220'])
    # print(dict_from_csv[13]['voxel_23220'])
    print(dict_from_csv[14]['voxel_23220'])

    # print(dict_from_csv.keys())


# Pytorch_Kinematics to get homogeneous transformation matrix
def pytorch_kinematics_implementation () :
    # Still waiting for implementation...
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


# Input of randomized points, which will be converted to voxels, search dictionary for collision status
def main_calculation () :
    # Still waiting for implementation...
    dict = {}



if __name__ == "__main__":
    dev = "cpu"
    dtype = torch.float32
    read_dict()