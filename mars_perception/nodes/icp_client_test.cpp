#include <mars_msgs/ICPMeshTF.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc,argv, "icp_client");

    ros::NodeHandle n("perception");
    ros::ServiceClient client = n.serviceClient<mars_msgs::ICPMeshTF>("icp_mesh_tf");

    client.waitForExistence();

    mars_msgs::ICPMeshTF srv;

    std::string mesh_name,base_frame;
    ros::param::get("~mesh_name",mesh_name);
    ros::param::get("~base_link",base_frame);

    srv.request.mesh_name = mesh_name;
    if (client.call(srv))
    {
        geometry_msgs::PoseStamped p;
        p.pose = srv.response.tf.pose;
        p.header.stamp = ros::Time::now(); 
        p.header.frame_id = base_frame; 
        ROS_INFO("pos: x,y,z: %f,%f,%f", p.pose.position.x, p.pose.position.y, p.pose.position.z);
        ROS_INFO("ori:x,y,z,w: %f,%f,%f,%f", p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
    }
    else
    {
        std::cout << "here" << "\n";
        ROS_ERROR("Failed to call service icp_mesh_tf");
    }
    ros::spin();
    return 0;
}