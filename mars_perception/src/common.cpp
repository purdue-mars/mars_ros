#include <mars_perception/common.h>

PointCloudPtr MeshUtil::get_pc_ptr()
{
    return mesh_pc_;
}

std::string MeshUtil::get_name()
{
    return mesh_name_;
}

void MeshUtil::stl_to_pcl_(std::string mesh_path)
{
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileOBJ(mesh_path,mesh);
    polygon_mesh_to_pc(&mesh, mesh_pc_);
}

bool MeshUtil::update_mesh(std::string name)
{
    mesh_name_ = name;
    std::string mesh_param_name = "mesh_directory/" + name;
    if (!ros::param::has(mesh_param_name))
    {
        ROS_ERROR("Mesh directory not correctly initialized or mesh does not exist in mesh_directory! %s", mesh_param_name.c_str());
        return false;
    }

    std::string mesh_path;
    ros::param::get(mesh_param_name, mesh_path);
    ROS_INFO("MESH_PATH: %s", mesh_path.c_str());
    try
    {
        stl_to_pcl_(mesh_path);
    }
    catch (std::exception &e)
    {
        ROS_ERROR("%s\n",e.what());
        return false;
    }
    return true;
}