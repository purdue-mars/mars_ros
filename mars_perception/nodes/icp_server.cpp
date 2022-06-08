#include <mars_perception/icp.h>

int main(int argc, char** argv) {
    ros::init(argc,argv, "icp_server");
    ICP icp;
    while(ros::ok()) {
        ros::spinOnce();
        icp.run();
    }
}