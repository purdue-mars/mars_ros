#include <mars_perception/icp.h>

ICP::ICP() : scene_pc_(new PointCloud), tf_(TFMatrix::Identity())
{
    ros::param::param<double>("~max_correspondence_distance", max_corresp_dist_, 0.5);
    ros::param::param<double>("~transformation_epsilon", transf_epsilon_, 1e-11);
    ros::param::param<double>("~fitness_epsilon", fitness_epsilon_, 1e-3);
    ros::param::param<double>("~max_iterations", max_iter_, 100);
    ros::param::get("base_link", base_frame_);

    std::string scene_pc_topic;
    ros::param::get("~filtered_points_topic", scene_pc_topic);

    icp_mesh_srv_ = nh_.advertiseService("icp_mesh_tf", &ICP::mesh_icp_srv, this);
    mesh_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("object_mesh_pc", 10);
    scene_pc_sub_ = nh_.subscribe(scene_pc_topic, 10, &ICP::scene_pc_cb_, this);
}

void ICP::scene_pc_cb_(const PointCloudMsg::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, *scene_pc_);
}

void ICP::run()
{
    try
    {
        PointCloudPtr mesh_ptr = mesh_.get_pc_ptr();
        if (scene_pc_->empty() || mesh_ptr->empty())
        {
            return;
        }
        pcl::IterativeClosestPoint<Point, Point> icp;
        icp.setInputSource(mesh_ptr);
        icp.setInputTarget(scene_pc_);

        icp.align(*mesh_ptr);

        tf_ = icp.getFinalTransformation() * tf_;

        tf::Transform transform;
        std::string frame_id = mesh_.get_name();
        geometry_msgs::TransformStamped tf;
        Eigen::Quaternionf q(tf_.topLeftCorner<3, 3>());
        transform.setOrigin(tf::Vector3(tf_.col(3)(0), tf_.col(3)(1), tf_.col(3)(2)));
        transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        tf.child_frame_id = frame_id;
        tf.header.frame_id = base_frame_;
        tf.header.stamp = ros::Time::now();
        tf.transform.translation.x = tf_.col(3)(0);
        tf.transform.translation.y = tf_.col(3)(1);
        tf.transform.translation.z = tf_.col(3)(2);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        br_.sendTransform(tf);

        mesh_ptr->header.frame_id = base_frame_;
        mesh_pub_.publish(mesh_ptr);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

bool ICP::mesh_icp_srv(mars_msgs::PointCorrTF::Request &req, mars_msgs::PointCorrTF::Response &resp)
{
    bool result = mesh_.update_mesh(req.mesh_name);
    if(!result) {
        ROS_ERROR("Update mesh failed! Double check that mesh .obj file is valid!");
        return false;
    }

    tf_ = TFMatrix::Identity();

    for (int i = 0; i < 10; i++)
    {
        run();
    }
    resp.tf.header.frame_id = req.mesh_name;
    resp.tf.header.stamp = ros::Time::now();
    Eigen::Quaternionf q(tf_.topLeftCorner<3, 3>());
    resp.tf.pose.position.x = tf_.col(3)(0);
    resp.tf.pose.position.y = tf_.col(3)(1);
    resp.tf.pose.position.z = tf_.col(3)(2);
    resp.tf.pose.orientation.x = q.x();
    resp.tf.pose.orientation.y = q.y();
    resp.tf.pose.orientation.z = q.z();
    resp.tf.pose.orientation.w = q.w();
    return true;
}