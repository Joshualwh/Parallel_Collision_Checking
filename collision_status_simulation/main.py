from mesh_to_sdf import get_surface_point_cloud
from urdf_parser_py.urdf import URDF
from trimesh.voxel import creation
from trimesh.viewer import *
from trimesh.collision import CollisionManager
from trimesh.creation import *

import pandas as pd
import torch
import trimesh
import numpy as np

# Read excel file, create dictionary 
# Pytorch_Kinematics to get homogeneous transformation matrix
# Input of randomized points, which will be converted to voxels, search dictionary for collision status




if __name__ == "__main__":