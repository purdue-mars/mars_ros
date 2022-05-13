#include <mars_perception/icp.h>

ICP::ICP() : filter_(), mesh_pc(new PointCloud) {

    ros::param::get("~max_correspondence_distance", max_corresp_dist_);
    ros::param::get("~transformation_epsilon", transf_epsilon);
    ros::param::get("~fitness_epsilon", fitness_epsilon);
    ros::param::get("~max_iterations", max_iter);

    set_mesh_(DEFAULT_MESH);
    wait_for_scene_point_cloud();
    icp_mesh_srv = nh_.advertiseService("icp_mesh_tf", &ICP::mesh_icp_srv, this);
}

void ICP::set_mesh_(std::string mesh_name) {
    assert(ros::param::has(mesh_name));
    std::string mesh_path;
    nh_.getParam(mesh_name,mesh_path);
    std::cout << "Mesh: " << mesh_path << "\n";
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(mesh_path,mesh);
    pcl::fromPCLPointCloud2(mesh.cloud,*mesh_pc);
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
    return true;
}