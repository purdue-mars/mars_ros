#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <mars_msgs/ICPMeshTF.h>
#include <mars_msgs/MoveToAction.h>

int main(int argc, char** argv) {
    ros::init(argc,argv, "pick_node");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mars_msgs::ICPMeshTF>("icp_mesh_tf");
    actionlib::SimpleActionClient<mars_msgs::MoveToAction> move_to_serv("move_to", true);

    tf::TransformListener tf_listener;

    // Get pose of object
    client.waitForExistence();

    mars_msgs::ICPMeshTF srv;
    std::string mesh_name = "large_round_peg";
    geometry_msgs::PoseStamped p;

    std::string base_name;
    ros::param::get("/base_name",base_name);
    srv.request.mesh_name = mesh_name;

    if (!client.call(srv))
    {
        std::cout << "here" << "\n";
        ROS_ERROR("Failed to call service icp_mesh_tf");
        return 0;
    }

    tf::StampedTransform transform;

    try {
        tf_listener.lookupTransform(base_name, srv.response.tf.header.frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // Move to object
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    move_to_serv.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    mars_msgs::MoveToGoal goal;

    goal.target.position.x = transform.getOrigin().x(); 
    goal.target.position.y = transform.getOrigin().y(); 
    goal.target.position.z = transform.getOrigin().z(); 
    move_to_serv.sendGoal(goal);

    ros::spin();
    return 0;
}