from mesh_to_sdf import mesh_to_voxels, mesh_to_sdf
from mesh_to_sdf import get_surface_point_cloud, sample_sdf_near_surface, scale_to_unit_cube

import trimesh
import skimage
import pyrender
import numpy as np
import os

#Voxelize a mesh
def Voxelize():
    mesh = trimesh.load("meshes/link_1.obj")
    voxels = mesh_to_voxels(mesh, 64, pad=True)
    vertices, faces, normals, _ = skimage.measure.marching_cubes(voxels, level=0)
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, vertex_normals=normals)
    mesh.show()

#Sample SDF points non-uniformly near the surface
def Sample_SDF_Points():
    mesh = trimesh.load('meshes/link_1.obj')
    mesh = scale_to_unit_cube(mesh)
    points, sdf = sample_sdf_near_surface(mesh, number_of_points=250000)
    print(sdf)
    colors = np.zeros(points.shape)
    colors[sdf < 0, 2] = 1
    colors[sdf > 0, 0] = 1
    cloud = pyrender.Mesh.from_points(points, colors=colors)
    scene = pyrender.Scene()
    scene.add(cloud)
    viewer = pyrender.Viewer(scene, use_raymond_lighting=True, point_size=2)

def self_test() :
    mesh = trimesh.load('meshes/link_2.obj')
    mesh = scale_to_unit_cube(mesh)

    print("Scanning...")
    cloud = get_surface_point_cloud(mesh, surface_point_method='scan', scan_count=20, scan_resolution=400)

    # cloud.show()

    # os.makedirs("test", exist_ok=True)
    # for i, scan in enumerate(cloud.scans):
    #     scan.save("test/scan_{:d}.png".format(i))

    query_points = np.array([[2,1,1]])
    point = mesh_to_sdf(mesh, query_points)

    print(point)

    # print("Voxelizing...")
    # voxels = cloud.get_voxels(128, use_depth_buffer=True)

    # print("Creating a mesh using Marching Cubes...")
    # vertices, faces, normals, _ = skimage.measure.marching_cubes(voxels, level=0)
    # mesh = trimesh.Trimesh(vertices=vertices, faces=faces, vertex_normals=normals)
    # mesh.show()

if __name__ == "__main__":
    # Voxelize()
    Sample_SDF_Points()
    self_test()