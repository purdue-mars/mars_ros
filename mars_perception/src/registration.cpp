#include <mars_perception/registration.h>

PCRegistration::PCRegistration() : nh_(), tf_listener_(), cloud_concatenated_(new PointCloudT)
{
  ros::param::get("/filtered_output", output_topic_);

  ros::param::get("~input_topics", input_topics_);
  ros::param::get("~concat_frame", concat_frame_id_);
  ros::param::get("~box_min", box_min_);
  ros::param::get("~box_max", box_max_);
  ros::param::get("~leaf_sizes", leaf_sizes_);

  ros::param::get("~max_correspondence_distance", max_corresp_dist_);
  ros::param::get("~transformation_epsilon", transf_epsilon_);
  ros::param::get("~fitness_epsilon", fitness_epsilon_);
  ros::param::get("~max_iterations", max_iter_);
  ros::param::get("~ransac_rejection_threshold", reject_thres_);

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
      SyncPolicyT(10), *cloud_subscribers_[0], *cloud_subscribers_[1], *cloud_subscribers_[2]);

  cloud_synchronizer_->registerCallback(
      boost::bind(&PCRegistration::pointcloud_callback, this, _1, _2, _3));
  cloud_publisher_ = nh_.advertise<PointCloudMsgT>(output_topic_, 1);
}

void PCRegistration::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2, const PointCloudMsgT::ConstPtr &msg3)
{

  PointCloudMsgT::ConstPtr msgs[PC_SIZE] = {msg1, msg2, msg3};
  PointCloudT::Ptr cloud_sources[PC_SIZE];

  cloud_concatenated_ = PointCloudT::Ptr(new PointCloudT);

  // transform points
  try
  {
    for (size_t i = 0; i < input_topics_.size(); ++i)
    {
      cloud_sources[i] = PointCloudT().makeShared();
      pcl::fromROSMsg(*msgs[i], *cloud_sources[i]);
      tf_listener_.waitForTransform(concat_frame_id_, msgs[i]->header.frame_id, ros::Time(0), ros::Duration(1.0));
      pcl_ros::transformPointCloud(concat_frame_id_, ros::Time(0), *cloud_sources[i], msgs[i]->header.frame_id, *cloud_sources[i], tf_listener_);

      if (cloud_sources[i]->size() != 0)
      {
        pcl::VoxelGrid<PointT> voxel_filter;
        pcl::CropBox<PointT> box_filter;
        box_filter.setInputCloud(cloud_sources[i]);

        box_filter.setMin(Eigen::Vector4f(box_min_[0], box_min_[1], box_min_[2], 1.0));
        box_filter.setMax(Eigen::Vector4f(box_max_[0], box_max_[1], box_max_[2], 1.0));
        box_filter.filter(*cloud_sources[i]);

        voxel_filter.setInputCloud(cloud_sources[i]);
        voxel_filter.setLeafSize((double)leaf_sizes_[i][0], (double)leaf_sizes_[i][1], (double)leaf_sizes_[i][2]);
        voxel_filter.filter(*cloud_sources[i]);
      }

      // double mean, stddev;
      // ros::param::get("~outlier_mean", mean);
      // ros::param::get("~outlier_stddev", stddev);
      // pcl::StatisticalOutlierRemoval<PointT> sor;
      // sor.setInputCloud(cloud_sources[i]);
      // sor.setMeanK(mean);
      // sor.setStddevMulThresh(stddev);
      // sor.filter(*cloud_sources[i]);
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
    if (cloud_sources[i]->size() != 0)
    {
      if (i != 0)
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
      *cloud_concatenated_ += *cloud_sources[i];
    }
  }
  // Create the filtering object
  // pcl::CropBox<PointT> box_filter;
  // box_filter.setInputCloud(cloud_concatenated_);

  // std::vector<double> box_min, box_max;
  // ros::param::get("~box_min", box_min);
  // ros::param::get("~box_max", box_max);
  // box_filter.setMin(Eigen::Vector4f(box_min[0], box_min[1], box_min[2], 1.0));
  // box_filter.setMax(Eigen::Vector4f(box_max[0], box_max[1], box_max[2], 1.0));
  // box_filter.filter(*cloud_concatenated_);

  // double mean, stddev;
  // ros::param::get("~outlier_mean", mean);
  // ros::param::get("~outlier_stddev", stddev);
  // pcl::StatisticalOutlierRemoval<PointT> sor;
  // sor.setInputCloud(cloud_concatenated_);
  // sor.setMeanK(mean);
  // sor.setStddevMulThresh(stddev);
  // sor.filter(*cloud_concatenated_);

  // Voxel grid filter
  // pcl::ApproximateVoxelGrid<PointT> voxel_filter;
  // voxel_filter.setInputCloud(cloud_concatenated_);
  // voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f);
  // voxel_filter.filter(*cloud_concatenated_);

  // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // // Create the segmentation object
  // pcl::SACSegmentation<PointT> seg;
  // // Optional
  // seg.setOptimizeCoefficients (true);
  // // Mandatory
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setDistanceThreshold (0.011);

  // seg.setInputCloud (cloud_concatenated_);
  // seg.segment (*inliers, *coefficients);

  // // Create the filtering object
  // pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  // extract.setInputCloud (cloud_concatenated_);
  // extract.setIndices (inliers);
  // extract.setNegative (true);
  // extract.filter (* cloud_concatenated_);

  // Publish
  cloud_concatenated_->header = pcl_conversions::toPCL(msgs[0]->header);
  cloud_concatenated_->header.frame_id = concat_frame_id_;
  cloud_publisher_.publish(cloud_concatenated_);
}