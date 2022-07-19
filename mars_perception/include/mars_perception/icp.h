#pragma once

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <bits/stdc++.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mars_msgs/PointCorrTF.h>
#include <mars_perception/mesh_sampling.h>
#include <mars_perception/common.h>

class ICP
{
public:
    ICP();
    bool mesh_icp_srv(mars_msgs::PointCorrTF::Request &req, mars_msgs::PointCorrTF::Response &resp);
    void run();

private:
    MeshUtil mesh_;
    PointCloudPtr scene_pc_;
    ros::NodeHandle nh_;
    ros::ServiceServer icp_mesh_srv_;
    ros::Publisher mesh_pub_;
    ros::Subscriber scene_pc_sub_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster br_;

    std::string base_frame_;
    TFMatrix tf_;

    double max_corresp_dist_;
    double transf_epsilon_;
    double fitness_epsilon_;
    double max_iter_;

    void scene_pc_cb_(const PointCloudMsg::ConstPtr &msg);
};
