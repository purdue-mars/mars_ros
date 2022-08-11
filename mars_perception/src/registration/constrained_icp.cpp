#include <mars_perception/registration/constrained_icp.h>

const std::string ConstrainedICP::NAME = "CONSTRAINED_ICP";

void ConstrainedICP::set_axes(int rot_i, int trans0_i, int trans1_i) {
    assert(rot_i > 0 && rot_i < 3);
    assert(trans0_i > 0 && trans0_i < 3);
    assert(trans1_i > 0 && trans1_i < 3);
    rot_axis_ = Eigen::Vector3d::Zero();
    rot_axis_[rot_i] = 1;
    trans_axis0_ = Eigen::Vector3d::Zero();
    trans_axis0_[trans0_i] = 1;
    trans_axis1_ = Eigen::Vector3d::Zero();
    trans_axis1_[trans1_i] = 1;
}

void crop_cloud_by_depth(open3d::geometry::PointCloud& pcd, float depth)
{
    std::vector<Eigen::Vector3d> cropped_points;
    std::vector<Eigen::Vector3d> cropped_colors;

    for (int i = 0; i < pcd.points_.size(); ++i)
    {
        if (pcd.points_[i].z() < depth)
        {
            cropped_points.push_back(pcd.points_[i]);
            cropped_colors.push_back(pcd.colors_[i]);
        }
    }

    pcd.points_ = std::move(cropped_points);
    pcd.colors_ = std::move(cropped_colors);
}

void remove_cloud_outliers(open3d::geometry::PointCloud& pcd, int num_points, double radius, double ratio)
{
    std::shared_ptr<open3d::geometry::PointCloud> cloud;
    std::vector<size_t> indices;
    std::tie(cloud, indices) = pcd.RemoveRadiusOutliers(num_points, radius);
    pcd.SelectByIndex(indices);

    if (ratio > 0)
    {
        std::tie(cloud, indices) = pcd.RemoveStatisticalOutliers(50, ratio);
        pcd.SelectByIndex(indices);
    }
}

void ConstrainedICP::constrained_icp_single_step_(open3d::geometry::PointCloud& source, open3d::geometry::PointCloud& target, 
    Eigen::Matrix4d& base_transform, double max_correspond_dist, double* x)
{
    int num_points = source.points_.size();
    ConstrainedICPCost* icp_cost = new ConstrainedICPCost(
                source, target, base_transform, rot_axis_, trans_axis0_, trans_axis1_, max_correspond_dist);
    ceres::LossFunction* loss_function = new ceres::SoftLOneLoss(1.0);
    // ceres::LossFunction* loss_function = nullptr;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    // options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1;

    ceres::Problem problem;
    ceres::Solver::Summary summary;
    ceres::CostFunction* cost_function = 
        new ceres::NumericDiffCostFunction<ConstrainedICPCost, ceres::FORWARD, ceres::DYNAMIC, 3>(icp_cost, ceres::TAKE_OWNERSHIP, num_points);
    problem.AddResidualBlock(cost_function, loss_function, x);

    ceres::Solve(options, &problem, &summary);
}

std::pair<Eigen::Matrix4d, Eigen::Matrix6d>
ConstrainedICP::pairwise_registration_(open3d::geometry::PointCloud& source, open3d::geometry::PointCloud& target, Eigen::Matrix4d& base_transform)
{   
    std::cout << "Apply constrained point-to-plane ICP..." << std::endl;
    
    Eigen::Matrix4d transform = base_transform;

    double x[3] = {0, 0, 0};
    open3d::geometry::PointCloud pcd = source;
    pcd.Transform(base_transform);

    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();

    for (int i = 0; i < 5; ++i)
    {
        constrained_icp_single_step_(pcd, target, I, dist_coarse_, x);

        double angle = x[0];
        double t0 = x[1];
        double t1 = x[2];

        Eigen::Matrix4d update = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd angle_axis(angle, rot_axis_);
        update.block<3, 3>(0, 0) = angle_axis.matrix();
        update.block<3, 1>(0, 3) = t0 * trans_axis0_ + t1 * trans_axis1_;
        pcd.Transform(update);
        transform = update * transform;
    }
    
    for (int i = 0; i < 10; ++i)
    {
        x[0] = x[1] = x[2] = 0.0;
        constrained_icp_single_step_(pcd, target, I, dist_fine_, x);

        double angle = x[0];
        double t0 = x[1];
        double t1 = x[2];

        // Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d update = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd angle_axis(angle, rot_axis_);
        update.block<3, 3>(0, 0) = angle_axis.matrix();
        update.block<3, 1>(0, 3) = t0 * trans_axis0_ + t1 * trans_axis1_;
        pcd.Transform(update);
        transform = update * transform;
    }

    // constrained_icp(source, target, base_transform, dist_COARSE, x);
    // constrained_icp(source, target, base_transform, dist_FINE, x);

    Eigen::Matrix6d information = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(source, target, dist_fine_, transform);

    return std::move(std::make_pair(transform, information));
}

open3d::pipelines::registration::PoseGraph ConstrainedICP::build_pose_graph_(std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& pcds, std::vector<Eigen::Matrix4d>& poses,
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>>& geometries)
{
    open3d::pipelines::registration::PoseGraph pose_graph;
    pose_graph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(poses[0]));

    Eigen::Matrix4d base_transform = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transform;
    Eigen::Matrix6d information;
    for (int i = 1; i < poses.size(); ++i)
    {
        pose_graph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(poses[i]));

        std::tie(transform, information) = pairwise_registration_(*pcds[i-1], *pcds[i], base_transform);

        // Just for debug
        // pcds[i-1]->Transform(transform);
        // auto g = std::vector<std::shared_ptr<const open3d::geometry::Geometry>>{geometries[i-1], geometries[i]};
        // open3d::DrawGeometries(g);

        pose_graph.edges_.push_back(open3d::pipelines::registration::PoseGraphEdge(i-1, i, transform, information, false));
    }

    std::tie(transform, information) = pairwise_registration_(*pcds.back(), *pcds[0], base_transform);
    pose_graph.edges_.push_back(open3d::pipelines::registration::PoseGraphEdge(pcds.size()-1, 0, transform, information, true));

    return std::move(pose_graph);
}

void ConstrainedICP::preprocess_(std::shared_ptr<open3d::geometry::PointCloud> pcd) {
    crop_cloud_by_depth(*pcd, max_depth_);
    remove_cloud_outliers(*pcd, 30, voxel_size_, 1);
    pcd->VoxelDownSample(voxel_size_);
    pcd->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size_ * 5, 30));
}

void ConstrainedICP::run() {

    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> pcds;

    std::vector<Eigen::Matrix4d> poses;
    poses.push_back(Eigen::Matrix4d::Identity());
    poses.push_back(tf_mat_ptr->cast<double>());

    pcl_to_open3d(scene_ptr, *scene_o3d_);
    pcl_to_open3d(mesh_ptr, *mesh_o3d_);

    geometries.push_back(scene_o3d_);
    pcds.push_back(scene_o3d_);

    geometries.push_back(mesh_o3d_);
    pcds.push_back(mesh_o3d_);

    open3d::pipelines::registration::PoseGraph pose_graph = build_pose_graph_(pcds, poses, geometries);
    open3d::pipelines::registration::GlobalOptimizationOption option(dist_fine_, 0.25, 1.0, 0);
    open3d::pipelines::registration::GlobalOptimization(
        pose_graph, 
        open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt(), 
        open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria(),
        option);
    
    for (int i = 0; i < poses.size(); ++i)
    {
        pcds[i]->Transform(pose_graph.nodes_[i].pose_);
    }

    *tf_mat_ptr = pose_graph.nodes_[1].pose_.cast<float>();
}
