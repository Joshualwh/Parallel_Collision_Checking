# -*- coding: utf-8 -*-

# from openravepy import *
import numpy as np
import torch

class VoxelConversion(object):
    def __init__(self, gridcenter, gridextent, numvoxels):
        gridcenter.clone().detach()
        gridextent.clone().detach()
        numvoxels.clone().detach()
        self.origin = gridcenter - gridextent
        self.numvoxels = numvoxels
        self.voxelsize = 2 * gridextent / numvoxels
        self.halfvoxelsize = self.voxelsize / 2

    def toIndex(self, point):
        t = point - self.halfvoxelsize - self.origin
        i = (t / self.voxelsize)
        i = i.short()
# The error Message Bool value of Tensor with more than one value is ambiguous appears when you try to cast a tensor into a bool value. This happens most commonly when passing the tensor to an if condition, e.g.
        i = i.numpy()
        # self.numvoxels = self.numvoxels.numpy()
        print (i)

        if i[0] >= 0 and i[0] < self.numvoxels[0]:
            index = i[2] + self.numvoxels[2] * i[1] + self.numvoxels[2] * self.numvoxels[1] * i[0]
            print(index)
            return index
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
    from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())

