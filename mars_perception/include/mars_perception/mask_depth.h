#pragma once
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>

#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <detectron2_ros/Result.h>
#include <cv_bridge/cv_bridge.h>
#include <depth_image_proc/depth_traits.h>

#define CAM_CNT 3
class MaskDepth
{

public:
    MaskDepth();

private:
    typedef pcl::PointXYZRGB PointT;
    typedef sensor_msgs::Image ImageT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef sensor_msgs::PointCloud2 PointCloudMsgT;
    typedef detectron2_ros::Result Result;
    typedef message_filters::sync_policies::ApproximateTime<ImageT, ImageT, ImageT>
        SyncPolicyT;

    ros::NodeHandle nh_;
    message_filters::Subscriber<ImageT> *depth_subs_[CAM_CNT];
    message_filters::Synchronizer<SyncPolicyT> *depth_sync_;

    message_filters::Subscriber<Result> *mask_subs_[CAM_CNT];
    message_filters::Synchronizer<SyncPolicyT> *mask_sync_;

    cv::Mat masks_[CAM_CNT];
    XmlRpc::XmlRpcValue img_sizes_;

    ros::Subscriber config_subscriber_;
    ros::Publisher cloud_publisher_;
    tf::TransformListener tf_listener_;

    XmlRpc::XmlRpcValue leaf_sizes_;
    std::vector<double> box_min_, box_max_;

    double max_corresp_dist_;
    double transf_epsilon_;
    double fitness_epsilon_;
    double max_iter_;
    double reject_thres_;

    std::string base_frame_id_;

    PointCloudT::Ptr concat_masked_cloud_;

    void depth_image_cb(const ImageT::ConstPtr &msg1, const ImageT::ConstPtr &msg2, const ImageT::ConstPtr &msg3);
    void mask_cb(const Result::ConstPtr &msg1, const Result::ConstPtr &msg2, const Result::ConstPtr &msg3);

    void depth_to_pointcloud(const sensor_msgs::ImageConstPtr& depth_msg,
                                        const sensor_msgs::ImageConstPtr& rgb_msg,
                                        const PointCloud::Ptr& cloud_msg);
};