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
#include <mars_msgs/ICPMeshTF.h>
#include <mars_perception/mesh_sampling.h>

#define ICP_CONVERGE_SLEEP_TIME 1.5

class ICP
{
public:
    typedef pcl::PointXYZRGB Point;
    typedef pcl::PointCloud<Point> PointCloud;
    typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
    typedef sensor_msgs::PointCloud2 PointCloudMsg;
    typedef Eigen::Matrix4f TFMatrix;
    ICP();
    bool mesh_icp_srv(mars_msgs::ICPMeshTF::Request &req, mars_msgs::ICPMeshTF::Response &resp);
    void run();
private:
    PointCloudPtr mesh_pc_;
    PointCloudPtr scene_pc_;
    ros::NodeHandle nh_;
    ros::ServiceServer icp_mesh_srv_;
    ros::Publisher mesh_pub_;
    ros::Subscriber scene_pc_sub_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster br_;

    std::string mesh_name_;
    std::string base_frame_;
    TFMatrix tf_;
    double max_corresp_dist_;
    double transf_epsilon_;
    double fitness_epsilon_;
    double max_iter_;

    void set_mesh_(std::string);
    void scene_pc_cb_(const PointCloudMsg::ConstPtr& msg);

};
