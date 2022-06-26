#include <mars_perception/mask_depth.h>

MaskDepth::MaskDepth() : nh_(), tf_listener_(), concat_masked_cloud_(new PointCloudT)
{
    // global
    ros::param::get("/base_frame", base_frame_id_);

    std::vector<std::string> cameras_ns;
    ros::param::get("/camera_ns", cameras_ns);

    std::string depth_topic;
    std::vector<std::string> depth_topics;
    ros::param::get("/depth_topic", depth_topic);
    for (std::string t : cameras_ns)
        depth_topics.push_back(t + depth_topic);

    std::string color_topic;
    ros::param::get("/color_topic", color_topic);
    for (std::string t : cameras_ns)
        depth_topics.push_back(t + color_topic);

    std::string mask_topic;
    std::vector<std::string> mask_topics;
    ros::param::get("/mask_topic", mask_topic);
    for (std::string t : cameras_ns)
        mask_topics.push_back(t + mask_topic);

    std::string masked_points_topic; 
    ros::param::get("/masked_points_topic", masked_points_topic);
    ros::param::get("/camera_image_sizes", img_sizes_);

    // box filter params
    ros::param::get("~box_min", box_min_);
    ros::param::get("~box_max", box_max_);
    ros::param::get("~leaf_sizes", leaf_sizes_);

    // ICP params
    ros::param::get("~max_correspondence_distance", max_corresp_dist_);
    ros::param::get("~transformation_epsilon", transf_epsilon_);
    ros::param::get("~fitness_epsilon", fitness_epsilon_);
    ros::param::get("~max_iterations", max_iter_);
    ros::param::get("~ransac_rejection_threshold", reject_thres_);

    if (depth_topics.size() != CAM_CNT || mask_topics.size() != CAM_CNT)
    {
        ROS_ERROR("The size of camera_topics must be between 2");
        ros::shutdown();
    }

    for (size_t i = 0; i < CAM_CNT; ++i)
    {
        depth_subs_[i] =
            new message_filters::Subscriber<ImageT>(nh_, depth_topics[i], 10);
    }
    depth_sync_ = new message_filters::Synchronizer<SyncPolicyT>(
        SyncPolicyT(10), *depth_subs_[0], *depth_subs_[1], *depth_subs_[2]);
    depth_sync_->registerCallback(
        boost::bind(&MaskDepth::depth_image_cb, this, _1, _2, _3));

    for (size_t i = 0; i < CAM_CNT; ++i)
    {
        color_subs_[i] =
            new message_filters::Subscriber<ImageT>(nh_, color_topics[i], 10);
    }
    color_sync_ = new message_filters::Synchronizer<SyncPolicyT>(
        SyncPolicyT(10), *color_subs_[0], *color_subs_[1], *color_subs_[2]);
    color_sync_->registerCallback(
        boost::bind(&MaskDepth::color_image_cb, this, _1, _2, _3));

    for (size_t i = 0; i < CAM_CNT; ++i)
    {
        info_subs_[i] =
            new message_filters::Subscriber<ImageT>(nh_, color_topics[i], 10);
    }
    color_sync_ = new message_filters::Synchronizer<SyncPolicyT>(
        SyncPolicyT(10), *color_subs_[0], *color_subs_[1], *color_subs_[2]);
    color_sync_->registerCallback(
        boost::bind(&MaskDepth::color_image_cb, this, _1, _2, _3));

    for (size_t i = 0; i < CAM_CNT; ++i)
    {
        mask_subs_[i] =
            new message_filters::Subscriber<Result>(nh_, mask_topics[i], 10);
    }
    mask_sync_ = new message_filters::Synchronizer<SyncPolicyT>(
        SyncPolicyT(10), *mask_subs_[0], *mask_subs_[1], *mask_subs_[2]);
    mask_sync_->registerCallback(
        boost::bind(&MaskDepth::mask_cb, this, _1, _2, _3));

    cloud_publisher_ = nh_.advertise<PointCloudMsgT>(masked_points_topic, 1);
}

void MaskDepth::mask_cb(const Result::ConstPtr &msg1, const Result::ConstPtr &msg2, const Result::ConstPtr &msg3)
{
    std::vector<ImageT> masks[CAM_CNT] = {msg1->masks, msg2->masks, msg3->masks};
    for(int i = 0; i < masks.size(); i++) {
        masks_[i] = cv::Mat::zeros(cv::Size((double)img_sizes_[i][0],(double)img_sizes_[i][1]), CV_8UC3);

        for(int j = 0; i < masks[i].size(); j++) {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(masks[i][j]);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            masks_[i] = cv::bitwise_or(cv_ptr->image,masks_[i]);
        }
    } 
}

void MaskDepth::info_cb(const InfoT::ConstPtr &msg1, const InfoT::ConstPtr &msg2, const InfoT::ConstPtr &msg3)
{
    std::vector<ImageT> masks[CAM_CNT] = {msg1->masks, msg2->masks, msg3->masks};
    for(int i = 0; i < masks.size(); i++) {
        masks_[i] = cv::Mat::zeros(cv::Size((double)img_sizes_[i][0],(double)img_sizes_[i][1]), CV_8UC3);

        for(int j = 0; i < masks[i].size(); j++) {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(masks[i][j]);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            masks_[i] = cv::bitwise_or(cv_ptr->image,masks_[i]);
        }
    } 
}

void MaskDepth::depth_image_cb(const Result::ConstPtr &msg1, const Result::ConstPtr &msg2, const Result::ConstPtr &msg3)
{
    ImageT depth[CAM_CNT] = {msg1->masks, msg2->masks, msg3->masks};
    for(int i = 0; i < depth.size(); i++) {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(depth[i][j]);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::bitwise_and(cv_ptr->image,masks_[i]);
        masked_depth_[i] = cv_ptr->toImageMsg();
    } 
}

void MaskDepth::color_image_cb(const Result::ConstPtr &msg1, const Result::ConstPtr &msg2, const Result::ConstPtr &msg3)
{
    ImageT color[CAM_CNT] = {msg1->masks, msg2->masks, msg3->masks};
    for(int i = 0; i < color.size(); i++) {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(color[i][j]);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::bitwise_and(cv_ptr->image, masks_[i]);
        masked_color_[i] = cv_ptr->toImageMsg();
    } 
}

void MaskDepth::convert()
{
    PointCloudT::Ptr masked_clouds[CAM_CNT];
    concat_masked_cloud_ = PointCloudT::Ptr(new PointCloudT);

    // to point cloud 
    try
    {
        for (size_t i = 0; i < CAM_CNT; ++i)
        {
            masked_clouds[i] = PointCloudT().makeShared();
            depth_to_pointcloud(masked_depth_[i], masked_color_[i], masked_clouds[i])

            tf_listener_.waitForTransform(base_frame_id_, masked_depth_[i].header.frame_id, ros::Time(0), ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_id_, ros::Time(0), *masked_clouds[i], msgs[i]->header.frame_id, *masked_clouds[i], tf_listener_);

            // if (masked_clouds[i]->size() != 0)
            // {
            //     pcl::VoxelGrid<PointT> voxel_filter;
            //     pcl::CropBox<PointT> box_filter;
            //     box_filter.setInputCloud(masked_clouds[i]);

            //     box_filter.setMin(Eigen::Vector4f(box_min_[0], box_min_[1], box_min_[2], 1.0));
            //     box_filter.setMax(Eigen::Vector4f(box_max_[0], box_max_[1], box_max_[2], 1.0));
            //     box_filter.filter(*masked_clouds[i]);

            //     voxel_filter.setInputCloud(masked_clouds[i]);
            //     voxel_filter.setLeafSize((double)leaf_sizes_[i][0], (double)leaf_sizes_[i][1], (double)leaf_sizes_[i][2]);
            //     voxel_filter.filter(*masked_clouds[i]);
            // }

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
        if (masked_clouds_[i]->size() != 0)
        {
            if (i != 0)
            {
                pcl::IterativeClosestPoint<PointT, PointT> icp;
                icp.setInputSource(masked_clouds[i]);
                icp.setInputTarget(masked_clouds[0]);
                icp.setMaxCorrespondenceDistance(max_corresp_dist_);
                icp.setMaximumIterations(max_iter_);
                icp.setTransformationEpsilon(transf_epsilon_);
                icp.setRANSACOutlierRejectionThreshold(reject_thres_);
                icp.setEuclideanFitnessEpsilon(fitness_epsilon_);
                icp.align(*masked_clouds[i]);
            }
            *concat_masked_cloud_ += *masked_clouds[i];
        }
    }
    // Publish
    concat_masked_cloud_->header = pcl_conversions::toPCL(masked_depth_[0].header);
    concat_masked_cloud_->header.frame_id = base_frame_id_;
    cloud_publisher_.publish(concat_masked_cloud_);
}


void MaskDepth::depth_to_pointcloud(const sensor_msgs::ImageConstPtr& depth_msg,
                                      const sensor_msgs::ImageConstPtr& rgb_msg,
                                      const PointCloudT::Ptr& cloud_msg, 
                                      const image_geometry::PinholeModel& model) {
                                           // Use correct principal point from calibration
  float center_x = model.cx();
  float center_y = model.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / model.fx();
  float constant_y = unit_scaling / model.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  const uint8_t* rgb = &rgb_msg->data[0];
  int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(*cloud_msg, "a");

  for (int v = 0; v < int(cloud_msg->height); ++v, depth_row += row_step, rgb += RGB8_COLOR_STEP)
  {
    for (int u = 0; u < int(cloud_msg->width); ++u, rgb += color_step, ++iter_x, ++iter_y, ++iter_z, ++iter_a, ++iter_r, ++iter_g, ++iter_b)
    {
      T depth = depth_row[u];

      // Check for invalid measurements
      if (!DepthTraits<T>::valid(depth))
      {
        *iter_x = *iter_y = *iter_z = bad_point;
      }
      else
      {
        // Fill in XYZ
        *iter_x = (u - center_x) * depth * constant_x;
        *iter_y = (v - center_y) * depth * constant_y;
        *iter_z = DepthTraits<T>::toMeters(depth);
      }

      // Fill in color
      *iter_a = 255;
      *iter_r = rgb[RGB8_RED_OFFSET];
      *iter_g = rgb[RGB8_GREEN_OFFSET];
      *iter_b = rgb[RGB8_BLUE_OFFSET];
    }
  }
}
