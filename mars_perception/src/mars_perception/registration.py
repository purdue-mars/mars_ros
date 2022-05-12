import numpy as np
from sensor_msgs.point_cloud2 import PointCloud2 
from pyflann import *

# Pose Estimation (requires correspondences)

def EstimatePose(p_s, p_m, corresp):
    
    central_p_s = np.mean(p_s,axis=0)
    central_p_m = np.mean(p_m,axis=0)

    W = (p_m - central_p_m).T @ (p_s - central_p_s)

    # Compute R
    U, Sigma, Vt = np.linalg.svd(W)
    R = np.matmul(U, Vt)
    if np.linalg.det(R) < 0:
       print("fixing improper rotation")
       Vt[-1, :] *= -1
       R = np.matmul(U, Vt)

    # Compute p
    p = central_p_s - np.matmul(R, central_p_m)

    return p,R


# ICP

def FindClosestPoints(pc_a, pc_b):
    """
    Finds the nearest (Euclidean) neighbor in point_cloud_B for each
    point in point_cloud_A.
    @param point_cloud_A A 3xN numpy array of points.
    @param point_cloud_B A 3xN numpy array of points.
    @return indices An (N, ) numpy array of the indices in point_cloud_B of each
        point_cloud_A point's nearest neighbor.
    """
    indices = np.empty(pc_a.shape[1], dtype=int)

    result, dists = flann.nn(
    dataset, testset, 2, algorithm="kmeans", branching=32, iterations=7, checks=16)

    # TODO(russt): Replace this with a direct call to flann
    # https://pypi.org/project/flann/
    kdtree = open3d.geometry.KDTreeFlann(point_cloud_B)
    for i in range(point_cloud_A.shape[1]):
        nn = kdtree.search_knn_vector_3d(point_cloud_A[:,i], 1)
        indices[i] = nn[1][0]

    return indices

def ICP(p_s, p_m):

    corresp = np.zeros(p_s.shape[1])
