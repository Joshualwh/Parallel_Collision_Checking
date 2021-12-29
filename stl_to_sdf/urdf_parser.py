from mesh_to_sdf import mesh_to_voxels, mesh_to_sdf
from mesh_to_sdf import get_surface_point_cloud, sample_sdf_near_surface, scale_to_unit_cube

import trimesh

from urdf_parser_py.urdf import URDF

file = "osr_description/urdf/denso_vs060.urdf"
robot = URDF.from_xml_file(file)
links = robot.links
n_links = len(links)
# for i in range(n_links):
#     print(robot.links[i].collision.geometry.filename)
robot_mesh_filename = robot.links[0].collision.geometry.filename
found = robot_mesh_filename.replace('package://', '')

print(found)

mesh = trimesh.load_mesh(found)
print("Scanning...")
cloud = get_surface_point_cloud(mesh, surface_point_method='scan', scan_count=20, scan_resolution=400)

cloud.show()