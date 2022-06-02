#include <mars_perception/registration.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_registration");
    PCRegistration preprocess;
    ros::spin();
}