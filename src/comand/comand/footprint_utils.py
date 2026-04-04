#------------------------------------#
# FOR GETING THE 5 SHAPED WALL SHAPE #
#------------------------------------#

import trimesh
import alphashape
from shapely.geometry import Polygon, Point


def compute_footprint(mesh_path: str, alpha: float = 0.1):
    ''' 
    Description
        compute the 2D footprint of a 3D object represented by a mesh -> (mesh is provided via a file path)
        the function computes an alpha shape (generalized form of the convex hull that can handle concavities in the shape) 

    ---------
    Inputs:
        mesh_path (str): 
            the path to the mesh file (e.g., .stl, .obj). 
            the file contains the 3D geometry of the object.

        alpha (float): optional
            controls the "tightness" of the alpha shape. 
            smaller values of alpha create a more detailed shape (with more concavities), while larger values produce a more convex shape.
    --------
    Output:
        footprint(shapely.geometry.Polygon):
            returns a 2D polygon that represents the footprint of the object. 
            the polygon is computed by projecting the 3D mesh onto the XY plane and applying the alpha shape algorithm to the projected points.
            ---if the alpha shape cannot produce a valid polygon, the function raises a RuntimeError---
    '''

    #load mesh using the path
    mesh = trimesh.load(mesh_path)

    # Project vertices to XY
    points_2d = mesh.vertices[:, :2]

    # Compute alpha shape
    footprint = alphashape.alphashape(points_2d, alpha)
    # check i is a valid polygon
    if footprint.geom_type == 'Polygon':
        return footprint
    else:
        raise RuntimeError("Alpha shape did not produce a valid polygon")