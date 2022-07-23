#include <mars_perception/pc_processor.h>

PointCloudProcesser::PointCloudProcesser() : nh_(), tf_listener_(), cloud_concatenated(new PointCloudT)
{
  std::vector<std::string> point_cloud_topics;
  std::string output_topic;

  ros::param::get("base_link", base_frame_);

  // global
  ros::param::get("~filtered_points_topic", output_topic);
  ros::param::get("~point_cloud_topics", point_cloud_topics);

  // box filter params
  ros::param::get("~box_enabled", box_enabled_);
  ros::param::get("~box_min", box_min_);
  ros::param::get("~box_max", box_max_);

  // voxel filter params
  ros::param::get("~voxel_enabled", voxel_enabled_);
  ros::param::get("~leaf_sizes", leaf_sizes_);

  // ICP params
  ros::param::get("~icp_enabled", icp_enabled_);
  ros::param::get("~max_correspondence_distance", max_corresp_dist_);
  ros::param::get("~transformation_epsilon", transf_epsilon_);
  ros::param::get("~fitness_epsilon", fitness_epsilon_);
  ros::param::get("~max_iterations", max_iter_);
  ros::param::get("~ransac_rejection_threshold", reject_thres_);


  int topic_cnt = point_cloud_topics.size();
  if (topic_cnt > CAM_CNT || topic_cnt == 0)
  {
    ros::shutdown();
  }
  else if (topic_cnt < CAM_CNT)
  {

    // fills in rest of topics with first topic
    // TODO: restructure to dynamically load topics
    for (int i = 0; i < CAM_CNT - topic_cnt; i++)
    {
      std::string s = point_cloud_topics[0];
      point_cloud_topics.push_back(s);
    }
  }

  for (size_t i = 0; i < CAM_CNT; ++i)
  {
    cloud_subscribers_[i] =
        new message_filters::Subscriber<PointCloudMsgT>(nh_, point_cloud_topics[i], 10);
  }

  cloud_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(
      SyncPolicyT(10), *cloud_subscribers_[0], *cloud_subscribers_[1], *cloud_subscribers_[2]);

  cloud_synchronizer_->registerCallback(
      boost::bind(&PointCloudProcesser::pointcloud_callback, this, _1, _2, _3));
  cloud_publisher_ = nh_.advertise<PointCloudMsgT>(output_topic, 1);
}

void PointCloudProcesser::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2, const PointCloudMsgT::ConstPtr &msg3)
{

  PointCloudMsgT::ConstPtr msgs[CAM_CNT] = {msg1, msg2, msg3};
  PointCloudT::Ptr cloud_sources[CAM_CNT];

  cloud_concatenated = PointCloudT::Ptr(new PointCloudT);

  // transform points
  try
  {
    for (size_t i = 0; i < CAM_CNT; ++i)
    {
      cloud_sources[i] = PointCloudT().makeShared();
      pcl::fromROSMsg(*msgs[i], *cloud_sources[i]);
      tf_listener_.waitForTransform(base_frame_, msgs[i]->header.frame_id, ros::Time(0), ros::Duration(1.0));
      pcl_ros::transformPointCloud(base_frame_, ros::Time(0), *cloud_sources[i], msgs[i]->header.frame_id, *cloud_sources[i], tf_listener_);

      if (cloud_sources[i]->size() != 0)
      {
        pcl::CropBox<PointT> box_filter;
        box_filter.setInputCloud(cloud_sources[i]);
        box_filter.setMin(Eigen::Vector4f(box_min_[0], box_min_[1], box_min_[2], 1.0));
        box_filter.setMax(Eigen::Vector4f(box_max_[0], box_max_[1], box_max_[2], 1.0));
        box_filter.filter(*cloud_sources[i]);

        // pcl::MedianFilter<PointT> median_filter;
        // median_filter.setInputCloud(cloud_sources[i]);
        // median_filter.filter(*cloud_sources[i]);

        if (voxel_enabled_)
        {
          pcl::VoxelGrid<PointT> voxel_filter;
          voxel_filter.setInputCloud(cloud_sources[i]);
          voxel_filter.setLeafSize((double)leaf_sizes_[i][0], (double)leaf_sizes_[i][1], (double)leaf_sizes_[i][2]);
          voxel_filter.filter(*cloud_sources[i]);
        }
      }
    }
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // merge points
  for (size_t i = 0; i < CAM_CNT; ++i)
  {
    if (cloud_sources[i]->size() != 0)
    {
      if (icp_enabled_ && i != 0)
      {
        try
        {
          if (icp_enabled_)
          {
            pcl::IterativeClosestPoint<PointT, PointT> icp;
            icp.setInputSource(cloud_sources[i]);
            icp.setInputTarget(cloud_sources[0]);
            icp.setMaxCorrespondenceDistance(max_corresp_dist_);
            icp.setMaximumIterations(max_iter_);
            icp.setTransformationEpsilon(transf_epsilon_);
            icp.setRANSACOutlierRejectionThreshold(reject_thres_);
            icp.setEuclideanFitnessEpsilon(fitness_epsilon_);
            icp.align(*cloud_sources[i]);
          }
        }
        catch (const std::exception &e)
        {
          std::cerr << e.what() << '\n';
        }
      }
      *cloud_concatenated += *cloud_sources[i];
    }
  }

  // Publish
  cloud_concatenated->header = pcl_conversions::toPCL(msgs[0]->header);
  cloud_concatenated->header.frame_id = base_frame_;
  cloud_publisher_.publish(cloud_concatenated);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_processor");
    PointCloudProcesser processor;
    ros::spin();
}