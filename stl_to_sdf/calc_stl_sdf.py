from mesh_to_sdf import mesh_to_voxels, mesh_to_sdf
from mesh_to_sdf import get_surface_point_cloud, sample_sdf_near_surface, scale_to_unit_cube

import trimesh
import skimage
import pyrender
import numpy as np
import os

def calc() :
    mesh = trimesh.load_mesh('urdf/meshes/BASE.stl')
    mesh = scale_to_unit_cube(mesh)

    print("Scanning...")
    cloud = get_surface_point_cloud(mesh, surface_point_method='scan', scan_count=20, scan_resolution=400)

    cloud.show()

if __name__ == "__main__":
    calc()