#include <mars_msgs/RegistrationSrv.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "registration_test");

    ros::NodeHandle n("perception");
    ros::ServiceClient client = n.serviceClient<mars_msgs::RegistrationSrv>("registration");

    client.waitForExistence();

    mars_msgs::RegistrationSrv srv;

    std::string mesh_name;
    ros::param::get("~mesh_name", mesh_name);

    srv.request.mesh_name = mesh_name;
    
    if (client.call(srv))
    {
        ros::Duration(3.0).sleep();
    }
    else
    {
        ROS_ERROR("Failed to call registration service!");
    }
    ros::spin();
    return 0;
}