#include <mars_perception/mask_depth.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_registration");
    MaskDepth depth_mask;
    ros::spin();
}