#include <mars_perception/pc_concat.h>
#include <bits/stdc++.h>
#include <Eigen/Dense>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/registration/icp.h>

const std::string DEFAULT_MESH = "square_peg";

class ICP {
    private:
        typedef pcl::PointXYZ Point;
        typedef pcl::PointCloud<Point> PointCloud;
        typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
        typedef Eigen::Matrix4f TFMatrix; 
        PointsConcatFilter filter_;
        PointCloudPtr mesh_pc;
        PointCloudPtr concat_pc;
        ros::NodeHandle nh_;
        void set_mesh_(std::string); 
    public:
        ICP();
        TFMatrix run_icp(std::string mesh_name);
};
