#include <mars_msgs/ICPMeshTF.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc,argv, "icp_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mars_msgs::ICPMeshTF>("icp_mesh_tf");

    client.waitForExistence();

    mars_msgs::ICPMeshTF srv;
    std::vector<std::string> mesh_names({"large_round_peg"});

    // for(int i = 0; i < mesh_names.size(); i++) {
    //     srv.request.mesh_name = mesh_names[i];
    //     if (client.call(srv))
    //     {
    //         geometry_msgs::PoseStamped p;
    //         p.pose = srv.response.tf.pose;
    //         p.header.stamp = ros::Time::now(); 
    //         p.header.frame_id = "panda_link0"; 
    //         ROS_INFO("pos: x,y,z: %f,%f,%f", p.pose.position.x, p.pose.position.y, p.pose.position.z);
    //         ROS_INFO("ori:x,y,z,w: %f,%f,%f,%f", p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
    //     }
    //     else
    //     {
    //         std::cout << "here" << "\n";
    //         ROS_ERROR("Failed to call service icp_mesh_tf");
    //     }
    // }
    ros::spin();
    return 0;
}