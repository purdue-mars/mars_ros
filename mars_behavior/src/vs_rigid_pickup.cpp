
/*
 * Implements pose-based visual servoing.
 *
 * Reference: https://link.springer.com/chapter/10.1007/978-3-319-32552-1_34#Sec12
 */

#include <ros/ros.h>
#include <mars_behavior/DoVSRigidPickupAction.h>
#include <actionlib/server/simple_action_server.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

typedef actionlib::SimpleActionServer<mars_behavior::DoVSRigidPickupAction> Server;

typedef Eigen::AngleAxisf AngleAxisf;
typedef Eigen::Matrix<float, 3, 3> Matrix3f;
typedef Eigen::Matrix<float, 4, 4> Matrix4f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Quaternionf Quaternionf;
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Vector<float, 6> Vector6f;

ros::Publisher cmd_pub;

float epsilon = 0.01;
Vector3f curr_pos; // in object frame
AngleAxisf curr_rot; // in object frame

void poseCallback(const geometry_msgs::Pose::ConstPtr &msg) {
    curr_pos << msg->position.x, msg->position.y, msg->position.z;
    Quaternionf quat << msg->orientation.w, msg->orientation.x,
        msg->orientation.y, msg->orientation.z;
    curr_rot = AngleAxisf(quat);
}

float sinc(float x) {
    if (x == 0) {
        return 1;
    }
    return std::sinf(x);
}

Matrix3f generateSkew(Vector3f v) {
  Matrix3f m;
  m << 0, -v(2), v(1),
       v(2), 0, -v(0),
       -v(1), v(0), 0;
  return m;
}

Matrix6f generateJacobian(Vector3f t, AngleAxisf r) {
    Matrix3f I = Matrix3f::Identity();
    Matrix3f Z = Matrix3f::Zero();
    Matrix3f s = generateSkew(r.Axis());
    Matrix3f R = Matrix3f::Identity() + ((r.Angle()/2.0) * s);
    R += 1 - (sinc(r.Angle())/(sinc(r.Angle()/2.0) * sinc(r.Angle()/2.0)) * s.dot(s));
    Matrix6f L << -I, generateSkew(t), Z, R;
    return L;
}

void execute(const mars_behavior::DoVSRigidGoalConstPtr& goal, Server* as) {
    ROS_INFO("Performing VSRigidPickup");

    Vector3f goal_pos = goal->target_position;
    Matrix6f L_inv;
    Vector6f err;
    Vector6f vel;

    ros::Rate r(10);
    while (ros::ok()) {
        L_inv = generateJacobian(curr_pos, curr_rot)
            .completeOrthogonalDecomposition().pseudoInverse();
        err << (goal_pos - curr_pos), curr_rot.Angle() * curr_rot.Axis();
        vel = -epsilon * (L_inv * err);
        
        geometry_msgs::Twist cmd;
        cmd.linear.x = vel(0);
        cmd.linear.y = vel(1);
        cmd.linear.z = vel(2);
        cmd.angular.x = vel(3);
        cmd.angular.y = vel(4);
        cmd.angular.z = vel(5);
        cmd_pub.publish(cmd);

        r.sleep();
    }

    as->setSucceeded();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "do_vs_rigid_pickup_server");
    
    ros::NodeHandle n;

    // Parameters
    std::string feat_topic, cmd_topic;
    if (!nh.getParam("~feature_topic", feat_topic)) {
        ROS_ERROR("No feature topic specified");
        return -1;
    } else if (!nh.getParam("~command_topic", cmd_topic)) {
        ROS_ERROR("No command topic specified");
        return -1;
    }

    // Subscribers
    n.subscribe(feat_topic, 1, featCallback);

    // Publishers
    cmd_pub = n.advertise<geometry_msgs::Twist>(cmd_topic, 1);

    // Action server
    Server server(n, "do_vs_rigid_pickup", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}