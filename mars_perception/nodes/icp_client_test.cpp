#include <mars_msgs/ICPMeshTF.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc,argv, "icp_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mars_msgs::ICPMeshTF>("icp_mesh_tf");

    client.waitForExistence();

    mars_msgs::ICPMeshTF srv;
    srv.request.mesh_name = "circle_peg";
    if (client.call(srv))
    {
        geometry_msgs::Pose p = srv.response.tf.pose;
        ROS_INFO("pos: x,y,z: %f,%f,%f", p.position.x, p.position.y, p.position.z);
        ROS_INFO("ori:x,y,z,w: %f,%f,%f,%f", p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    }
    else
    {
        std::cout << "here" << "\n";
        ROS_ERROR("Failed to call service icp_mesh_tf");
    }
    return 0;
}