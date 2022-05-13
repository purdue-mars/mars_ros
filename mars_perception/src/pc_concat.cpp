#include <mars_perception/pc_concat.h>

PointsConcatFilter::PointsConcatFilter() : nh_(), tf_listener_(), cloud_concatenated_(new PointCloudT)
{
  ros::param::get("~input_topics",input_topics_);
  ros::param::get("~filtered_output",output_topic_);
  ros::param::get("~concat_frame",concat_frame_id_);

  if (input_topics_.size() != PC_SIZE)
  {
    ROS_ERROR("The size of input_topics must be between 2");
    ros::shutdown();
  }

  for (size_t i = 0; i < PC_SIZE; ++i)
  {
      cloud_subscribers_[i] =
          new message_filters::Subscriber<PointCloudMsgT>(nh_, input_topics_[i], 10);
  }

  cloud_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(
      SyncPolicyT(10), *cloud_subscribers_[0], *cloud_subscribers_[1]);
      
  cloud_synchronizer_->registerCallback(
      boost::bind(&PointsConcatFilter::pointcloud_callback, this, _1, _2));
  cloud_publisher_ = nh_.advertise<PointCloudMsgT>(output_topic_, 1);
}

int PointsConcatFilter::empty() {
  return cloud_concatenated_.get()->empty();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointsConcatFilter::get_pointcloud_ptr() {
    return cloud_concatenated_;
}

void PointsConcatFilter::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2)
{

  PointCloudMsgT::ConstPtr msgs[2] = { msg1, msg2 };
  PointCloudT::Ptr cloud_sources[2];

  cloud_concatenated_ = PointCloudT::Ptr(new PointCloudT);

  // transform points
  try
  {
    for (size_t i = 0; i < input_topics_.size(); ++i)
    {
      // Note: If you use kinetic, you can directly receive messages as
      // PointCloutT.
      cloud_sources[i] = PointCloudT().makeShared();
      pcl::fromROSMsg(*msgs[i], *cloud_sources[i]);
      tf_listener_.waitForTransform(concat_frame_id_, msgs[i]->header.frame_id, ros::Time(0), ros::Duration(1.0));
      pcl_ros::transformPointCloud(concat_frame_id_, ros::Time(0), *cloud_sources[i], msgs[i]->header.frame_id, *cloud_sources[i], tf_listener_);
    }
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // merge points
  for (size_t i = 0; i < input_topics_.size(); ++i)
  {
    *cloud_concatenated_ += *cloud_sources[i];
  }
  // Create the filtering object
  pcl::CropBox<PointT> box_filter;
  box_filter.setInputCloud (cloud_concatenated_);
  box_filter.setMin(Eigen::Vector4f(0, -0.4064, 0, 1.0));
  box_filter.setMax(Eigen::Vector4f(0.9398, 0.4064, 0.20, 1.0));
  box_filter.filter(*cloud_concatenated_);


  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (cloud_concatenated_);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_concatenated_);

  // Publish
  cloud_concatenated_->header = pcl_conversions::toPCL(msgs[0]->header);
  cloud_concatenated_->header.frame_id = concat_frame_id_;
  cloud_publisher_.publish(cloud_concatenated_);
}