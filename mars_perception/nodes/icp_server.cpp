#include <mars_perception/icp.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "icp_server");
    ros::NodeHandle n;
    ros::Rate r(50);
    ICP icp;
    while(ros::ok()) {
        ros::spinOnce();
        icp.run();
        r.sleep();
    }
}