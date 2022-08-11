#pragma once

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <bits/stdc++.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <open3d/Open3D.h>

#include <mars_perception/mesh_sampling.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
typedef sensor_msgs::PointCloud2 PointCloudMsg;
typedef Eigen::Matrix4f TFMatrix;

class MeshUtil
{
public:
    std::string get_name();
    PointCloudPtr get_pc_ptr();
    bool update_mesh(std::string name, Eigen::Affine3d tf);
    MeshUtil() : mesh_pc_(new PointCloud) {}

private:
    PointCloudPtr mesh_pc_;
    std::string mesh_name_;
    void stl_to_pcl_(std::string mesh_path);
};

void pcl_to_open3d(PointCloudPtr pc_ptr, open3d::geometry::PointCloud& open3d_p);