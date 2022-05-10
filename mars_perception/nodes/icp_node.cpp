#include <mars_perception/icp.h>

int main(int argc, char** argv) {
    ros::init(argc,argv, "icp_node");
    ICP icp;
    std::cout << "here" << "\n";
    icp.run_icp("square_peg"); 
    ros::spin();
}