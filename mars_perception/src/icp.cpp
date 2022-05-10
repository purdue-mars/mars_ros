#include <mars_perception/icp.h>

ICP::ICP() : nh_(), filter_(&nh_) {
    std::cout << "entering set_mesh" << "\n";
    set_mesh_(DEFAULT_MESH);
    std::cout << "set_mesh" << "\n";
    concat_pc = filter_.get_pointcloud_ptr();
}

void ICP::set_mesh_(std::string mesh_name) {
    assert(ros::param::has(mesh_name));
    std::string mesh_path;
    nh_.getParam(mesh_name,mesh_path);
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(mesh_path,mesh);
    pcl::fromPCLPointCloud2(mesh.cloud,*mesh_pc);
}

ICP::TFMatrix ICP::run_icp(std::string mesh_name) {
    set_mesh_(mesh_name);
    pcl::IterativeClosestPoint<ICP::Point, ICP::Point> icp;
    icp.setInputSource(mesh_pc);
    icp.setInputTarget(concat_pc);
    
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    return icp.getFinalTransformation();
}