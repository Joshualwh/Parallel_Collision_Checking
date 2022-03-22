import torch

query_points_world_list = torch.zeros((64000, 4, 1), dtype=torch.float32, device='cpu')
x_count = 0
y_count = 0
z_count = 0
for not_sure_index in range (len(query_points_world_list)) :
    query_points_world_list[not_sure_index][0] = 0.04 * x_count
    query_points_world_list[not_sure_index][1] = 0.04 * y_count
    query_points_world_list[not_sure_index][2] = 0.04 * z_count
    if z_count == 40:
        y_count +=1
        z_count = 0
        if y_count == 40 :
            x_count +=1
            y_count = 0
    print(x_count, y_count, z_count)
    z_count +=1


query_points_world_list[:,3] = 1
print (query_points_world_list)