import torch

ten = torch.randn(4, 5)
ids = torch.tensor([2, 1, 4, 1])
mask = torch.ones_like(ten).scatter_(1, ids.unsqueeze(1), 0.)
res = ten[mask.bool()].view(4, 4)
res = ten[mask.bool()]

print(ten)
print(ids)

print(res)