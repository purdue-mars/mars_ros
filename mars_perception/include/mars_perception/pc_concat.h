/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>
#define PC_SIZE 2
class PointsConcatFilter
{
  public:
    PointsConcatFilter();
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_pointcloud_ptr();
    int empty();
  private:
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef sensor_msgs::PointCloud2 PointCloudMsgT;
    typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT>
        SyncPolicyT;

    ros::NodeHandle nh_;
    message_filters::Subscriber<PointCloudMsgT> *cloud_subscribers_[PC_SIZE];
    message_filters::Synchronizer<SyncPolicyT> *cloud_synchronizer_;
    ros::Subscriber config_subscriber_;
    ros::Publisher cloud_publisher_;
    tf::TransformListener tf_listener_;

    std::vector<std::string> input_topics_;
    std::string concat_frame_id_;

    PointCloudT::Ptr cloud_concatenated_;

    void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2);
};