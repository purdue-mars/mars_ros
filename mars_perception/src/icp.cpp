#include <mars_perception/icp.h>

ICP::ICP() : mesh_pc_(new PointCloud), scene_pc_(new PointCloud)
{
    ros::param::get("~max_correspondence_distance", max_corresp_dist_);
    ros::param::get("~transformation_epsilon", transf_epsilon_);
    ros::param::get("~fitness_epsilon", fitness_epsilon_);
    ros::param::get("~max_iterations", max_iter_);

    std::string scene_pc_topic;
    ros::param::get("filtered_points_topic", scene_pc_topic);

    set_mesh_(DEFAULT_MESH);
    wait_for_scene_point_cloud();
    icp_mesh_srv_ = nh_.advertiseService("icp_mesh_tf", &ICP::mesh_icp_srv, this);
    mesh_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("object_mesh_pc", 10);
    scene_pc_sub_ = nh_.subscribe(scene_pc_topic, 10, &ICP::scene_pc_cb_, this);
}

void ICP::scene_pc_cb_(const PointCloudMsg::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, *scene_pc_);
}

void ICP::set_mesh_(std::string mesh_name)
{
    assert(ros::param::has(mesh_name));
    std::string mesh_path;
    nh_.getParam(mesh_name, mesh_path);
    std::cout << "Mesh: " << mesh_path << "\n";
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(mesh_path, mesh);
    polygon_mesh_to_pc(&mesh, mesh_pc_);

    float centroid_x, centroid_y, centroid_z;

    centroid_x /= mesh_pc_->points.size();
    centroid_y /= mesh_pc_->points.size();
    centroid_z /= mesh_pc_->points.size();

    for (int i = 0; i < mesh_pc_->points.size(); i++)
    {
        mesh_pc_->points[i].x -= centroid_x;
        mesh_pc_->points[i].y -= centroid_y;
        mesh_pc_->points[i].z -= centroid_z;
    }

    std::cout << "mesh_frame: " << mesh_pc_->header.frame_id << "\n";
}

void ICP::icp(PointCloudPtr p1, PointCloudPtr p2, ICP::TFMatrix *tf)
{
    pcl::IterativeClosestPoint<ICP::Point, ICP::Point> icp;
    icp.setInputSource(p1);
    icp.setInputTarget(p2);
    icp.setMaxCorrespondenceDistance(max_corresp_dist_);
    icp.setTransformationEpsilon(transf_epsilon_);
    icp.setEuclideanFitnessEpsilon(fitness_epsilon_);
    icp.setMaximumIterations(max_iter_);

    pcl::PointCloud<Point> Final;
    icp.align(Final);

    std::cout << "has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    *tf = icp.getFinalTransformation();
}

void ICP::wait_for_scene_point_cloud()
{
    while (scene_pc_->empty())
    {
        ros::spinOnce();
    }
}

bool ICP::mesh_icp_srv(mars_msgs::ICPMeshTF::Request &req, mars_msgs::ICPMeshTF::Response &resp)
{

    ros::Rate r(5);
    if (scene_pc_->empty())
    {
        return false;
    }
    set_mesh_(req.mesh_name);
    ICP::TFMatrix tf;
    std::cout << "running icp"
              << "\n";
    icp(mesh_pc_, scene_pc_, &tf);
    resp.tf.header.frame_id = "panda_link0";
    resp.tf.header.stamp = ros::Time::now();
    Eigen::Quaternionf q(tf.topLeftCorner<3, 3>());
    resp.tf.pose.position.x = tf.col(3)(0);
    resp.tf.pose.position.y = tf.col(3)(1);
    resp.tf.pose.position.z = tf.col(3)(2);
    resp.tf.pose.orientation.x = q.x();
    resp.tf.pose.orientation.y = q.y();
    resp.tf.pose.orientation.z = q.z();
    resp.tf.pose.orientation.w = q.w();

    tf::Transform transform;
    std::string frame_id = req.mesh_name + "_frame";
    geometry_msgs::TransformStamped tf_;

    transform.setOrigin(tf::Vector3(tf.col(3)(0), tf.col(3)(1), tf.col(3)(2)));
    transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    tf_.child_frame_id = frame_id;
    tf_.header.frame_id = "panda_link0";
    tf_.header.stamp = ros::Time::now();
    tf_.transform.translation.x = tf.col(3)(0);
    tf_.transform.translation.y = tf.col(3)(1);
    tf_.transform.translation.z = tf.col(3)(2);
    tf_.transform.rotation.x = q.x();
    tf_.transform.rotation.y = q.y();
    tf_.transform.rotation.z = q.z();
    tf_.transform.rotation.w = q.w();
    br_.sendTransform(tf_);

    mesh_pc_->header.frame_id = frame_id;
    mesh_pub_.publish(*mesh_pc_);

    return true;
}