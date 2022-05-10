#include <mars_perception/pc_concat.h>
#include <bits/stdc++.h>
#include <Eigen/Dense>

class ICP {
    private:
        typedef pcl::PointXYZ Point;
        typedef pcl::PointCloud<Point> PointCloud;
        typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
        typedef pcl::registration<Point,Point,4>::TransformationEstimation::Matrix4 Matrix4; 
        PointsConcatFilter filter;
        PointCloudPtr mesh_cloud;
    public:
        ICP();
        Matrix4 icp();
};
