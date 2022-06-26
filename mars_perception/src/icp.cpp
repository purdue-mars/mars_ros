#include <mars_perception/icp.h>


ICP::ICP() : mesh_pc_(new PointCloud), scene_pc_(new PointCloud), tf_(TFMatrix::Identity())
{
    ros::param::get("~max_correspondence_distance", max_corresp_dist_);
    ros::param::get("~transformation_epsilon", transf_epsilon_);
    ros::param::get("~fitness_epsilon", fitness_epsilon_);
    ros::param::get("~max_iterations", max_iter_);
    ros::param::get("/base_frame", base_frame_);

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

void ICP::set_mesh_(std::string mesh_name)
{
    if(ros::param::has(mesh_name)) {
        std::string mesh_path;
        nh_.getParam(mesh_name, mesh_path);
        std::cout << "Mesh: " << mesh_path << "\n";
        pcl::PolygonMesh mesh;
        pcl::io::loadPolygonFileSTL(mesh_path, mesh);
        polygon_mesh_to_pc(&mesh, mesh_pc_);

        for (int i = 0; i < mesh_pc_->points.size(); i++)
        {
            mesh_pc_->points[i].x /= 1000;
            mesh_pc_->points[i].y /= 1000;
            mesh_pc_->points[i].z /= 1000;
        }

        float centroid_x = 0.0;
        float centroid_y = 0.0;
        float centroid_z = 0.0;

        for (int i = 0; i < mesh_pc_->points.size(); i++)
        {
        centroid_x += mesh_pc_->points[i].x;
        centroid_y += mesh_pc_->points[i].y;
        centroid_z += mesh_pc_->points[i].z;
        }

        centroid_x /= mesh_pc_->points.size();
        centroid_y /= mesh_pc_->points.size();
        centroid_z /= mesh_pc_->points.size();

        std::cout << centroid_x << " " << centroid_y << " " << centroid_z << "\n";

        for (int i = 0; i < mesh_pc_->points.size(); i++)
        {
            mesh_pc_->points[i].x -= centroid_x;
            mesh_pc_->points[i].y -= centroid_y;
            mesh_pc_->points[i].z -= centroid_z;
        }


        std::cout << "mesh_frame: " << mesh_pc_->header.frame_id << "\n";
    }
}

void ICP::run() {
    try
    {
        if (scene_pc_->empty() || mesh_pc_->empty())
        {
            return; 
        }
        pcl::IterativeClosestPoint<ICP::Point, ICP::Point> icp;
        icp.setInputSource(mesh_pc_);
        icp.setInputTarget(scene_pc_);

        icp.align(*mesh_pc_);

        tf_ = icp.getFinalTransformation() * tf_;

        tf::Transform transform;
        std::string frame_id = mesh_name_ + "_frame";
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

        mesh_pc_->header.frame_id = base_frame_;
        mesh_pub_.publish(mesh_pc_);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

bool ICP::mesh_icp_srv(mars_msgs::ICPMeshTF::Request &req, mars_msgs::ICPMeshTF::Response &resp)
{
    set_mesh_(req.mesh_name);
    tf_ = TFMatrix::Identity();
    mesh_name_ = req.mesh_name;
    std::cout << mesh_name_ << "\n"; 
    for(int i = 0; i < 10; i++) {
        run();
    }
    resp.tf.header.frame_id = mesh_name_ + "_frame";
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