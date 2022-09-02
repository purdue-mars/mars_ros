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

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
typedef sensor_msgs::PointCloud2 PointCloudMsg;
typedef Eigen::Matrix4f TFMatrix;

class MeshUtil
{
public:
    std::string get_name();
    PointCloudPtr get_pc_ptr();
    bool update_mesh(std::string name, TFMatrix tf);
    MeshUtil() : mesh_pc_(new PointCloud) {}

private:
    PointCloudPtr mesh_pc_;
    std::string mesh_name_;
};
