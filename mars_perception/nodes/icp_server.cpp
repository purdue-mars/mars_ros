#include <mars_perception/icp.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "icp_server");
    ros::NodeHandle n;
    ros::Rate r(50);
    ros::AsyncSpinner spinner(4);
    ICP icp;
    spinner.start();
    while(ros::ok()) {
        icp.run();
        r.sleep();
    }
}