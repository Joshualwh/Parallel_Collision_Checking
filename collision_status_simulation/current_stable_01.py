from urdf_parser_py.urdf import URDF

import pandas as pd
import torch
import pytorch_kinematics as pk
import time

# Create 7 dictionaries for collisions status of each voxel in each robot link
def read_dict () :

    # Import precomputed data and store in respective dictionaries
    dict_from_csv = pd.read_csv('trimesh_collision_status.csv', header=None, index_col=0, squeeze=True).to_dict()
    voxel_index = list(dict_from_csv[1].keys())
    voxel_index.pop(0)
    voxel_index = [int(x) for x in voxel_index]

    for i in range (len(voxel_index)) :
        link_0_dict[voxel_index[i]] = dict_from_csv[2][voxel_index[i]]
        link_1_dict[voxel_index[i]] = dict_from_csv[4][voxel_index[i]]
        link_2_dict[voxel_index[i]] = dict_from_csv[6][voxel_index[i]]
        link_3_dict[voxel_index[i]] = dict_from_csv[8][voxel_index[i]]
        link_4_dict[voxel_index[i]] = dict_from_csv[10][voxel_index[i]]
        link_5_dict[voxel_index[i]] = dict_from_csv[12][voxel_index[i]]
        link_6_dict[voxel_index[i]] = dict_from_csv[14][voxel_index[i]]

    print('Finished creating dict for each link...')

# Pytorch_Kinematics to obtain homogeneous transformation matrix
def pytorch_kinematics_implementation () :
    chain = pk.build_serial_chain_from_urdf(open("osr_description/urdf/denso_vs060.urdf").read(), "J6")
    # print(chain)
    
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

    # print("Randomized one FK and moving on...")
    return homogeneous_trans_mat

# Conversion of local coordinates to their respective voxel index
def toIndex (points) :
    voxelindices = torch.arange(40 ** 3).reshape(40, 40, 40).to(dev)

    # Details of the voxels
    voxelsize = 0.04
    numvoxel = 40
    workspacesize = voxelsize*numvoxel

    # Since the imported coordinates are randomized and might be out of our working range, we therefore filter the points in this process
    points_filtered = points[torch.all((points < workspacesize / 2) & (-workspacesize / 2 < points), dim=1)]
    origin = torch.tensor([-workspacesize/2,-workspacesize/2,-workspacesize/2], device=dev)

    # Voxel index is produced here
    voxelindices = ((points_filtered - origin) / voxelsize).to(torch.long)
    flattenedindices = voxelindices[:, 0]*numvoxel*numvoxel + voxelindices[:, 1]*numvoxel + voxelindices[:, 2]

    return flattenedindices  

# Input of randomized points, which will be converted to voxels, search dictionary for collision status
def main_calculation () :

    # Number of points for simulation
    num_of_points = 400000
    # num_of_points = 9000

    # Creation and transformation of randomized points 
    query_points_world_list = torch.rand((num_of_points, 4, 1), dtype=dtype, device=dev)
    query_points_world_list[:,3] = 1

    query_points_local_list = torch.matmul(homogeneous_trans_mat.repeat(len(query_points_world_list),1,1), query_points_world_list.repeat_interleave(len(homogeneous_trans_mat), dim=0))
    # print(query_points_local_list)
    query_points_local_list_reshaped = torch.squeeze(query_points_local_list)
    # print(query_points_local_list_reshaped)
    ids = torch.tensor([3], device=dev).repeat(7*num_of_points)

    mask = torch.ones_like(query_points_local_list_reshaped).scatter_(1, ids.unsqueeze(1), 0.)
    query_points_local_list_final = query_points_local_list[mask.bool()].view(7*num_of_points, 3)
    # print(query_points_local_list_final)
    
    # robot = URDF.from_xml_file(file)
    # links = robot.links
    n_links = 7
    
    # Main process of the simulation
    start_seconds = time.time()
    for links in range(n_links) :  
        vox_ind = toIndex(query_points_local_list_final[links*num_of_points:((links+1)*num_of_points)-1])
        # print(len(vox_ind))

        if n_links == 0:
            for i,number in enumerate (vox_ind):
                collision_status = link_0_dict[int(number)]
        elif n_links == 1:
            for i,number in enumerate (vox_ind):
                collision_status = link_0_dict[int(number)]
        elif n_links == 2:
            for i,number in enumerate (vox_ind):
                collision_status = link_0_dict[int(number)]
        elif n_links == 3:
            for i,number in enumerate (vox_ind):
                collision_status = link_0_dict[int(number)]
        elif n_links == 4:
            for i,number in enumerate (vox_ind):
                collision_status = link_0_dict[int(number)]
        elif n_links == 5:
            for i,number in enumerate (vox_ind):
                collision_status = link_0_dict[int(number)]
        elif n_links == 6:
            for i,number in enumerate (vox_ind):
                collision_status = link_0_dict[int(number)]

            # print(collision_status)
            # if collision_status == 'True' :
            #     print ('In Collision!')
            #     break
            # else :
            #     print ('All Cool!')

    time_taken = time.time() - start_seconds
    # print(time_taken)
    return len(vox_ind), time_taken


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
    pd_time_taken = 0
    pd_vox_ind = 0
    for i in range (100):
        homogeneous_trans_mat = pytorch_kinematics_implementation()
        vox_ind, time_taken = main_calculation()
        pd_time_taken += time_taken
        pd_vox_ind += vox_ind
    # print(pd_time_taken, pd_vox_ind)
    pd_time_taken = pd_time_taken/100
    pd_vox_ind = pd_vox_ind/100
    print(pd_time_taken, pd_vox_ind)
    # print(collision_status)
    # print(homogeneous_trans_mat)