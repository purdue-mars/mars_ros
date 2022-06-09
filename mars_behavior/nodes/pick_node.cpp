#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <mars_msgs/ICPMeshTF.h>
#include <mars_msgs/MoveToAction.h>

int main(int argc, char** argv) {
    ros::init(argc,argv, "pick_node");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mars_msgs::ICPMeshTF>("icp_mesh_tf");
    actionlib::SimpleActionClient<mars_msgs::MoveToAction> move_to_serv("move_to", true);

    // Get pose of object
    client.waitForExistence();

    mars_msgs::ICPMeshTF srv;
    std::string mesh_name = "large_round_peg";
    geometry_msgs::PoseStamped p;

    std::string base_name;
    ros::param::get("/base_name",base_name);
    srv.request.mesh_name = mesh_name;

    if (client.call(srv))
    {
        p.pose = srv.response.tf.pose;
        p.header.stamp = ros::Time::now(); 
        p.header.frame_id = base_name; 
        ROS_INFO("pos: x,y,z: %f,%f,%f", p.pose.position.x, p.pose.position.y, p.pose.position.z);
        ROS_INFO("ori:x,y,z,w: %f,%f,%f,%f", p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
    }
    else
    {
        std::cout << "here" << "\n";
        ROS_ERROR("Failed to call service icp_mesh_tf");
        return 0;
    }


    // Move to object
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    move_to_serv.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    mars_msgs::MoveToGoal goal;

    goal.target.position = p.pose.position; 
    move_to_serv.sendGoal(goal);

    ros::spin();
    return 0;
}