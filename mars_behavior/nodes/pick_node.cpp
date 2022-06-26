#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <mars_msgs/ICPMeshTF.h>
#include <mars_msgs/MoveToAction.h>
#include <sensor_msgs/JointState.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>
#include <franka_gripper/HomingAction.h>
#include <std_msgs/Float32.h>

#define GRAPS_THRES 0.15

class PickNode {

    public:

    ros::Subscriber gelsight_sub;
    ros::Subscriber gripper_joints_sub;
    ros::NodeHandle n;
    ros::ServiceClient client;
    actionlib::SimpleActionClient<mars_msgs::MoveToAction> move_to_act;
    actionlib::SimpleActionClient<franka_gripper::MoveAction> grip_move_act;
    actionlib::SimpleActionClient<franka_gripper::StopAction> grip_stop_act;
    actionlib::SimpleActionClient<franka_gripper::HomingAction> grip_home_act;
    void gelsight_cb(const std_msgs::Float32& msg);
    void gripper_joints_cb(const sensor_msgs::JointState& msg);
    tf::TransformListener tf_listener;

    float grasp_val;
    float gripper_width;
    float max_grip_width;
    geometry_msgs::Pose grasp_pose;

    PickNode() : move_to_act("move_to", true), grip_move_act("/franka_gripper/move", true), grip_stop_act("/franka_gripper/stop", true), grip_home_act("/franka_gripper/homing", true) {
        gelsight_sub = n.subscribe("/grasped",10,  &PickNode::gelsight_cb, this);
        gelsight_sub = n.subscribe("/franka_gripper/joint_states", 10,  &PickNode::gripper_joints_cb, this);
        client = n.serviceClient<mars_msgs::ICPMeshTF>("icp_mesh_tf");
        client.waitForExistence();
        move_to_act.waitForServer();
        grip_stop_act.waitForServer();
        grip_move_act.waitForServer();
        grip_home_act.waitForServer();
        franka_gripper::HomingGoal home_goal;
        grip_home_act.sendGoal(home_goal);
        grip_home_act.waitForResult();
        ros::spinOnce();
        max_grip_width = gripper_width;
        ROS_INFO("max_grip_width: %f", max_grip_width);
    }


    bool move_to_object() {
        // Get pose of object
        mars_msgs::ICPMeshTF srv;
        std::string mesh_name = "cable_male";
        geometry_msgs::PoseStamped p;

        std::string base_frame;
        ros::param::get("/base_frame",base_frame);
        srv.request.mesh_name = mesh_name;

        if (!client.call(srv))
        {
            std::cout << "here" << "\n";
            ROS_ERROR("Failed to call service icp_mesh_tf");
            return false;
        }

        tf::StampedTransform transform;

        ros::Duration(3.0).sleep();

        try {
            tf_listener.lookupTransform(base_frame, mesh_name + "_frame", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        // Move to object
        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        move_to_act.waitForServer(); //will wait for infinite time

        ROS_INFO("Action server started, sending goal.");
        // send a goal to the action
        mars_msgs::MoveToGoal goal;

        goal.target.position.x = transform.getOrigin().x(); 
        goal.target.position.y = transform.getOrigin().y(); 
        goal.target.position.z = transform.getOrigin().z() + 0.094 + 0.081; 
        goal.target.orientation.x = 1;
        goal.target.orientation.y = 0;
        goal.target.orientation.z = 0;
        goal.target.orientation.w = 0;
        grasp_pose = goal.target;
        move_to_act.sendGoal(goal);
        move_to_act.waitForResult();
    }

    void grasp() {
        franka_gripper::MoveGoal move_goal;
        franka_gripper::StopGoal stop_goal;
        move_goal.width = 0;
        move_goal.speed = 0.01;

        grip_move_act.sendGoal(move_goal);
        while(grasp_val < GRAPS_THRES) {
            ros::spinOnce();
            if(grip_move_act.getState() != actionlib::SimpleClientGoalState::PENDING && grip_move_act.getState() != actionlib::SimpleClientGoalState::ACTIVE) {
                ROS_INFO("ACTION CLIENT BREAK");
                break;
            }
        }
        grip_move_act.cancelAllGoals();
        grip_stop_act.sendGoal(stop_goal);
        ROS_INFO("GRASP COMPLETED!");
    }

    void go_up() {
        mars_msgs::MoveToGoal goal;
        goal.target = grasp_pose;
        goal.target.position.z += 0.1;
        move_to_act.sendGoal(goal);
        move_to_act.waitForResult();
    }
};

void PickNode::gelsight_cb(const std_msgs::Float32& msg) {
    grasp_val = msg.data;
}

void PickNode::gripper_joints_cb(const sensor_msgs::JointState& msg) {
    float width = 0;
    for(auto pos : msg.position) {
        width += pos;
    }
    gripper_width = width;
}

int main(int argc, char** argv) {
    ros::init(argc,argv, "pick_node");

    PickNode p;

    p.move_to_object();
    p.grasp();
    p.go_up();

    ROS_INFO("pick done!"); 
    return 0;
}