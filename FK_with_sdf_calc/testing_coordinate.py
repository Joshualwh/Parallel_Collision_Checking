import torch
from voxel_conversion import VoxelConversion

query_points_world_list = torch.zeros((64000, 4, 1), dtype=torch.float32, device='cpu')
x_count = 0
y_count = 0
z_count = 0
for not_sure_index in range (len(query_points_world_list)) :
    query_points_world_list[not_sure_index][0] = 0.04 * x_count - 0.78
    query_points_world_list[not_sure_index][1] = 0.04 * y_count - 0.78
    query_points_world_list[not_sure_index][2] = 0.04 * z_count - 0.78
    if z_count == 40:
        y_count +=1
        z_count = 0
        if y_count == 40 :
            x_count +=1
            y_count = 0
    # print(x_count, y_count, z_count)
    z_count +=1

gridcenter = torch.zeros(3, dtype=torch.float32, device='cpu')
gridextent = torch.ones(3, dtype=torch.float32) *0.8
numvoxels = torch.ones(3, dtype=torch.int16) *10

# gridcenter = np.array([0, 0, 0.4])
# gridextent = np.ones(3, dtype=np.float)*0.8
# numvoxels = np.ones(3, dtype=np.int16)*args.numvoxels

point = torch.tensor([0.5,0.,0.])

print (gridcenter, gridextent, numvoxels)

vc = VoxelConversion(gridcenter, gridextent, numvoxels)

vc.toIndex(point)


query_points_world_list[:,3] = 1
# print (query_points_world_list)