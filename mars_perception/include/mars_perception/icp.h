#pragma once
#include <mars_perception/pc_concat.h>
#include <bits/stdc++.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/registration/icp.h>
#include <mars_msgs/ICPMeshTF.h>
const std::string DEFAULT_MESH = "square_peg";

class ICP {
    private:
        typedef pcl::PointXYZ Point;
        typedef pcl::PointCloud<Point> PointCloud;
        typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
        PointsConcatFilter filter_;
        PointCloudPtr mesh_pc;
        ros::NodeHandle nh_;
        ros::ServiceServer icp_mesh_srv; 
        void set_mesh_(std::string); 
    public:
        typedef Eigen::Matrix4f TFMatrix; 
        ICP();
        bool mesh_icp_srv(mars_msgs::ICPMeshTF::Request &req, mars_msgs::ICPMeshTF::Response &resp);
        void icp(PointCloudPtr p1, PointCloudPtr p2, ICP::TFMatrix* tf);
        void wait_for_scene_point_cloud();
};
