#include <mars_perception/teaser.h>

Teaser::Teaser() : solver_(params_)
{
    teaser_registration_srv_ = nh_.advertiseService("teaser_registration", &Teaser::registration_srv, this);
    mesh_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("object_mesh", 10);

    std::string scene_pc_topic;
    ros::param::get("~filtered_points_topic", scene_pc_topic);

    scene_pc_sub_ = nh_.subscribe(scene_pc_topic, 10, &Teaser::scene_pc_cb_, this);
}

void Teaser::run()
{
    try
    {
        PointCloudPtr mesh_ptr = mesh_.get_pc_ptr();
        /* code */
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

bool Teaser::registration_srv(mars_msgs::RegistrationSrv::Request &req, mars_msgs::RegistrationSrv::Response &resp)
{
}

void Teaser::scene_pc_cb_()
{
}