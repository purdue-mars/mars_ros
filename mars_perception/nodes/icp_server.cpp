#include <mars_perception/icp.h>

int main(int argc, char** argv) {
    ros::init(argc,argv, "icp_server");
    ros::Rate r(20);
    ICP icp;
    while(ros::ok()) {
        ros::spinOnce();
        icp.run();
        r.sleep();
    }
}