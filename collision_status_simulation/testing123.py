import torch

coor_index = [47, 27, 30]
coor_index = torch.Tensor(coor_index)
print(coor_index)

numvoxels = [40, 40, 40]
numvoxels = torch.Tensor(numvoxels)
print(numvoxels)

checking_1 = coor_index >= torch.Tensor([0,0,0])
checking_2 = numvoxels >= coor_index


print (checking_2)
print (checking_2.any() == True)
print (checking_2.all() == True)