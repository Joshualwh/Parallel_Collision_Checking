# -*- coding: utf-8 -*-

# from openravepy import *
import numpy as np
import torch

class VoxelConversion(object):
    def __init__(self, gridcenter, gridextent, numvoxels):
        # gridcenter = gridcenter.clone().detach()
        # gridextent = gridextent.clone().detach()
        # numvoxels = numvoxels.clone().detach()
        self.origin = gridcenter - gridextent
        self.numvoxels = numvoxels
        self.voxelsize = 2 * gridextent / numvoxels
        self.halfvoxelsize = self.voxelsize / 2

    def toIndex_test (self, points) :
        voxelindices = torch.arange(40 ** 3).reshape(40, 40, 40)

        voxelsize = 0.04
        numvoxel = 40
        workspacesize = voxelsize*numvoxel

        points_filtered = points[torch.all((points < workspacesize / 2) & (-workspacesize / 2 < points), dim=1)]

        origin = torch.tensor([-workspacesize/2,-workspacesize/2,-workspacesize/2])

        # voxel = torch.tensor([voxelsize, voxelsize, voxelsize])
        voxelindices = ((points_filtered - origin) / voxelsize).to(torch.long)
        flattenedindices = voxelindices[:, 0]*numvoxel*numvoxel + voxelindices[:, 1]*numvoxel + voxelindices[:, 2]

        return flattenedindices  

    def toIndex(self, point):
        t = point - self.halfvoxelsize - self.origin
        coor_index = (t / self.voxelsize)
        coor_index = coor_index.short()

        checking_1 = coor_index >= torch.Tensor([0,0,0])
        checking_2 = self.numvoxels > coor_index

        if checking_1.all() == True and checking_2.all() == True :
            vox_con = torch.arange(40 ** 3).reshape(40, 40, 40)
            vox_ind = vox_con[coor_index[:, 0], coor_index[:, 1], coor_index[:, 2]]
            # print(vox_ind)
            return vox_ind.item()
        # self.numvoxels = self.numvoxels.numpy()
        # print (i)

        # if i[0] >= 0 and i[0] < self.numvoxels[0]:
        #     index = i[2] + self.numvoxels[2] * i[1] + self.numvoxels[2] * self.numvoxels[1] * i[0]
        #     # print(index)
        #     return index
        return -1

# class VoxelConversion(object):
#     def __init__(self, gridcenter, gridextent, numvoxels):
#         gridcenter = np.array(gridcenter)
#         gridextent = np.array(gridextent)
#         numvoxels = np.array(numvoxels, dtype=np.int32)
#         self.origin = gridcenter - gridextent
#         self.numvoxels = numvoxels
#         self.voxelsize = 2 * gridextent / numvoxels
#         self.halfvoxelsize = self.voxelsize / 2

#     def toIndex(self, point):
#         t = point - self.halfvoxelsize - self.origin
#         i = (t / self.voxelsize).astype(np.int16)

#         if i >= 0 and i < self.numvoxels:
#             index = i[2] + self.numvoxels[2] * i[1] + self.numvoxels[2] * self.numvoxels[1] * i[0]
#             return index
#         return -1

#     def toVoxelCenter(self, index):
#         assert(index >= 0 and index < np.prod(self.numvoxels))

#         k = index / (self.numvoxels[2] * self.numvoxels[1])
#         tmp = index % (self.numvoxels[2] * self.numvoxels[1])
#         j = tmp / self.numvoxels[2]
#         i = tmp % self.numvoxels[2]
#         pos = self.origin.copy() + self.halfvoxelsize
#         pos[0] += self.voxelsize[0] * k
#         pos[1] += self.voxelsize[1] * j
#         pos[2] += self.voxelsize[2] * i
#         return pos

#     def toVoxelCenter2(self, i, j, k):
#         pos = self.origin.copy() + self.halfvoxelsize
#         pos[0] += self.voxelsize[0] * i
#         pos[1] += self.voxelsize[1] * j
#         pos[2] += self.voxelsize[2] * k
#         return pos


if __name__ == '__main__':
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())

