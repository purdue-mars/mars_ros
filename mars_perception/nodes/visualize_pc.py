import numpy as np
from open3d import *    

def main():
    cloud = read_point_cloud("cloud.ply") # Read the point cloud
    draw_geometries([cloud]) # Visualize the point cloud     

if __name__ == "__main__":
    main()
