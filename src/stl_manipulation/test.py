import trimesh
import numpy as np
import os


_dir_path = os.path.dirname(os.path.realpath(__file__))
stl = os.path.join(_dir_path, "..", "..", "Scans", "Creality", "owl.stl")
ply_out = os.path.join(_dir_path, "..", "..", "Scans", "Creality", "owl.ply")

mesh = trimesh.load(stl)   # loads a mesh
mesh.export(ply_out)