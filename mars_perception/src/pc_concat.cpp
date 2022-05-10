#include <mars_perception/pc_concat.h>

PointsConcatFilter::PointsConcatFilter(ros::NodeHandle* nh) : nh_{nh}, tf_listener_()
{
  nh->getParam("~input_topics",input_topics_);
  nh->getParam("~concat_frame",concat_frame_id_);

  if (input_topics_.size() != PC_SIZE)
  {
    ROS_ERROR("The size of input_topics must be between 2");
    ros::shutdown();
  }
  for (size_t i = 0; i < PC_SIZE; ++i)
  {
    if (i < input_topics_.size())
    {
      cloud_subscribers_[i] =
          new message_filters::Subscriber<PointCloudMsgT>(*nh_, input_topics_[i], 1);
    }
    else
    {
      cloud_subscribers_[i] =
          new message_filters::Subscriber<PointCloudMsgT>(*nh_, input_topics_[0], 1);
    }
  }
  cloud_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(
      SyncPolicyT(10), *cloud_subscribers_[0], *cloud_subscribers_[1]);
      
  cloud_synchronizer_->registerCallback(
      boost::bind(&PointsConcatFilter::pointcloud_callback, this, _1, _2));
}

pcl::shared_ptr<PointsConcatFilter::PointCloudT> PointsConcatFilter::get_pointcloud_ptr() {
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
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_concat");
  PointsConcatFilter node;
  ros::spin();
  return 0;
}