import open3d

def point_cloud_from_mesh(filename, points):
    mesh = open3d.io.read_triangle_mesh(filename)
    open3d.geometry.sample_points_uniformly(mesh,points)
