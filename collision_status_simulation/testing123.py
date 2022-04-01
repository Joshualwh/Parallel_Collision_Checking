import torch

voxelindices = torch.arange(40 ** 3).reshape(40, 40, 40)
points = torch.randn(10, 3)
print(points)

voxelsize = 0.04
numvoxel = 40
workspacesize = voxelsize*numvoxel

points_filtered = points[torch.all((points < workspacesize / 2) & (-workspacesize / 2 < points), dim=1)]
print(points_filtered)

origin = -torch.tensor([workspacesize,workspacesize,workspacesize])/2

voxel = torch.tensor([voxelsize, voxelsize, voxelsize])
voxelindices = ((points_filtered - origin) / voxelsize).to(torch.long)
flattenedindices = voxelindices[:, 0]*numvoxel*numvoxel + voxelindices[:, 1]*numvoxel + voxelindices[:, 2]  

# correct?

# coor_index = [47, 27, 30]
# coor_index = torch.Tensor(coor_index)
# print(coor_index)

# numvoxels = [40, 40, 40]
# numvoxels = torch.Tensor(numvoxels)
# print(numvoxels)

# checking_1 = coor_index >= torch.Tensor([0,0,0])
# checking_2 = numvoxels >= coor_index


# print (checking_2)
# print (checking_2.any() == True)
# print (checking_2.all() == True)