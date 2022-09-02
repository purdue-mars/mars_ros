#pragma once

#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <open3d/Open3D.h>
#include <vector>
#include <open3d/pipelines/registration/GlobalOptimization.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/pipelines/registration/PoseGraph.h>

#include <mars_perception/registration/registration_base.h>

struct ConstrainedICPCost {

    ConstrainedICPCost(
        const open3d::geometry::PointCloud& source,
        const open3d::geometry::PointCloud& target,
        const Eigen::Matrix4d& _base_transform,
        const Eigen::Vector3d& _rotation_axis,
        const Eigen::Vector3d& _translation_axis0,
        const Eigen::Vector3d& _translation_axis1,
        double _max_correspond_dist
        )
        : pcd_source(source), pcd_target(target), base_transform(_base_transform),
        rotation_axis(_rotation_axis), translation_axis0(_translation_axis0), translation_axis1(_translation_axis1),
        max_correspond_dist(_max_correspond_dist)
    {
        kdtree.SetGeometry(pcd_target);
        int num_points = pcd_source.points_.size();
    }

    bool operator()(const double* const x, double* residual) const
    {
        if (!pcd_target.HasNormals()) return false;

        int num_points = pcd_source.points_.size();
        std::vector<Eigen::Vector3d> nearst_points(num_points);
        std::vector<Eigen::Vector3d> nearst_normals(num_points);

        std::vector<google::int32> indices(1);
        std::vector<double> distances(1);
        for (int i = 0; i < num_points; ++i)
        {
            indices.clear();
            distances.clear();
            int k = kdtree.SearchHybrid(pcd_source.points_[i], max_correspond_dist, 1, indices, distances);
            if (k == 1)
            {
                nearst_points[i] = pcd_target.points_[indices[0]];
                nearst_normals[i] = pcd_target.normals_[indices[0]];
            }
            else
            {
                nearst_points[i] = pcd_source.points_[i];
                nearst_normals[i] = Eigen::Vector3d::Zero();
            }
        }

        double angle = x[0];
        double t0 = x[1];
        double t1 = x[2];

        Eigen::AngleAxisd angle_axis(angle, rotation_axis);
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 3>(0, 0) = angle_axis.matrix();
        transform.block<3, 1>(0, 3) = t0 * translation_axis0 + t1 * translation_axis1;
        transform = transform * base_transform;

        for (int i = 0; i < num_points; ++i)
        {
            Eigen::Vector3d delta_position = 
                transform.block<3, 3>(0, 0) * pcd_source.points_[i] + transform.col(3).head(3) - nearst_points[i];
            residual[i] = delta_position.dot(nearst_normals[i]);
        }

        return true;
    }

    const open3d::geometry::PointCloud& pcd_source;
    const open3d::geometry::PointCloud& pcd_target;
    const Eigen::Matrix4d& base_transform;
    const Eigen::Vector3d& rotation_axis;
    const Eigen::Vector3d& translation_axis0;
    const Eigen::Vector3d& translation_axis1;
    double max_correspond_dist;
    open3d::geometry::KDTreeFlann kdtree;
};

class ConstrainedICP : public RegBase
{
public:
    static const std::string NAME;

    ConstrainedICP() : voxel_size_(0.02), max_depth_(2), rot_axis_(0,1,1), trans_axis0_(1,0,0), trans_axis1_(0,0,1),
        mesh_o3d_(std::make_shared<open3d::geometry::PointCloud>()),
        scene_o3d_(std::make_shared<open3d::geometry::PointCloud>()),
        RegBase() {
            dist_coarse_ = voxel_size_ * 15;
            dist_fine_ = voxel_size_ * 5;
        }
    void run();
    void set_axes(int rot_i, int trans0_i, int trans1_i);

private:
    double voxel_size_;
    double max_depth_;
    double dist_coarse_;
    double dist_fine_;
    Eigen::Vector3d rot_axis_;
    Eigen::Vector3d trans_axis0_;
    Eigen::Vector3d trans_axis1_;

    std::shared_ptr<open3d::geometry::PointCloud> mesh_o3d_; 
    std::shared_ptr<open3d::geometry::PointCloud> scene_o3d_; 

    void preprocess_(std::shared_ptr<open3d::geometry::PointCloud> pcd);

    std::pair<Eigen::Matrix4d, Eigen::Matrix6d>
    pairwise_registration_(open3d::geometry::PointCloud& source, open3d::geometry::PointCloud& target, Eigen::Matrix4d& base_transform);
    void constrained_icp_single_step_(open3d::geometry::PointCloud& source, open3d::geometry::PointCloud& target, 
        Eigen::Matrix4d& base_transform, double max_correspond_dist, double* x);

    open3d::pipelines::registration::PoseGraph build_pose_graph_(std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& pcds, std::vector<Eigen::Matrix4d>& poses,
        std::vector<std::shared_ptr<const open3d::geometry::Geometry>>& geometries);
};