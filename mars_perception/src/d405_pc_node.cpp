// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <mars_perception/rs_pcl.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>            // std::min, std::max

// Helper functions

int main(int argc, char * argv[]) try
{
    ros::init(argc,argv,"d405_pc_node");

    ros::NodeHandle nh;

    std::string pc2_out_topic; 
    std::string color_out_topic; 
    std::string serial_no; 
    ros::param::get("~pc2_out_topic",pc2_out_topic);
    ros::param::get("~color_out_topic",color_out_topic);
    ros::param::get("~serial_no",serial_no);

    ros::Publisher pcl_publisher = nh.advertise<sensor_msgs::PointCloud2>(pc2_out_topic, 1);;

    ros::Publisher color_publisher = nh.advertise<sensor_msgs::Image>(color_out_topic, 1);;
    
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    
    rs2::config cfg;
    cfg.enable_device(serial_no);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    ros::Rate r(30);
    pipe.start(cfg);
    while (ros::ok()) // Application still alive?
    {
        ros::spinOnce();
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto color = frames.get_color_frame();
        // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
        if (!color)
            color = frames.get_infrared_frame();

             // Creating OpenCV Matrix from a color image
        //Mat color(Size(640, 480), CV_8UC3, (void*)pipe->get_frame_data(rs::stream::color), Mat::AUTO_STEP);

        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        auto pcl_points = points_to_pcl(points);

        pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_points);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.0);
        pass.filter(*cloud_filtered);

        std::vector<pcl_ptr> layers;
        layers.push_back(pcl_points);
        layers.push_back(cloud_filtered);

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud_filtered.get(), msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "d405_frame"; 
        pcl_publisher.publish(msg);
        r.sleep();
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}