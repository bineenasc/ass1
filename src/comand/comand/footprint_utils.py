
import trimesh
import alphashape
from shapely.geometry import Polygon, Point


def compute_footprint(mesh_path: str, alpha: float = 0.1):
    mesh = trimesh.load(mesh_path)

    points_2d = mesh.vertices[:, :2]

    footprint = alphashape.alphashape(points_2d, alpha)
    if footprint.geom_type == 'Polygon':
        return footprint
    else:
        raise RuntimeError("Alpha shape did not produce a valid polygon")