import torch

T = torch.eye(4).repeat(7, 1, 1)
points = torch.randn(1000, 4, 1)
torch.matmul(T.repeat(len(points), 1, 1), points.repeat_interleave(len(T), dim=0)).shape