#include <mars_perception/icp.h>

ICP::ICP() : filter_(), mesh_pc(new PointCloud) {

    ros::param::get("~max_correspondence_distance", max_corresp_dist_);
    ros::param::get("~transformation_epsilon", transf_epsilon);
    ros::param::get("~fitness_epsilon", fitness_epsilon);
    ros::param::get("~max_iterations", max_iter);

    set_mesh_(DEFAULT_MESH);
    wait_for_scene_point_cloud();
    icp_mesh_srv = nh_.advertiseService("icp_mesh_tf", &ICP::mesh_icp_srv, this);
    mesh_pub = nh_.advertise<sensor_msgs::PointCloud2>("object_mesh_pc",10);
}

void ICP::set_mesh_(std::string mesh_name) {
    assert(ros::param::has(mesh_name));
    std::string mesh_path;
    nh_.getParam(mesh_name,mesh_path);
    std::cout << "Mesh: " << mesh_path << "\n";
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(mesh_path,mesh);
    polygon_mesh_to_pc(&mesh, mesh_pc);
    // mm to m
    for(int i = 0; i < mesh_pc->points.size(); i++) {
        mesh_pc->points[i].x /= 1000.0;
        mesh_pc->points[i].y /= 1000.0;
        mesh_pc->points[i].z /= 1000.0;
    }

    std::cout << "mesh_frame: " << mesh_pc->header.frame_id << "\n";

}

void ICP::icp(PointCloudPtr p1, PointCloudPtr p2, ICP::TFMatrix* tf) {
    pcl::IterativeClosestPoint<ICP::Point, ICP::Point> icp;
    icp.setInputSource(p1);
    icp.setInputTarget(p2);
    icp.setMaxCorrespondenceDistance(max_corresp_dist_);  
    icp.setTransformationEpsilon(transf_epsilon); 
    icp.setEuclideanFitnessEpsilon(fitness_epsilon); 
    icp.setMaximumIterations(max_iter);  
    
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    std::cout << "has converged: " << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    *tf = icp.getFinalTransformation();
}

void ICP::wait_for_scene_point_cloud() {
    while(filter_.empty()) {
        ros::spinOnce();
    }
}

bool ICP::mesh_icp_srv(mars_msgs::ICPMeshTF::Request &req, mars_msgs::ICPMeshTF::Response &resp) {

    ros::Rate r(5);
    if(filter_.empty()) {
        return false;
    }
    set_mesh_(req.mesh_name);
    ICP::TFMatrix tf;
    std::cout << "running icp" << "\n";
    icp(mesh_pc,filter_.get_pointcloud_ptr(),&tf);
    resp.tf.header.frame_id = "panda_link0";
    resp.tf.header.stamp = ros::Time::now();
    Eigen::Quaternionf q(tf.topLeftCorner<3,3>());
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

    transform.setOrigin( tf::Vector3(tf.col(3)(0),tf.col(3)(1),tf.col(3)(2)) );
    transform.setRotation( tf::Quaternion(q.x(),q.y(),q.z(),q.w()) );
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
    br.sendTransform(tf_);

    mesh_pc->header.frame_id = frame_id; 
    mesh_pub.publish(mesh_pc);

    return true;
}