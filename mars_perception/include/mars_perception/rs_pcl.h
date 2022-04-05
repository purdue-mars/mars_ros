// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#pragma once

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

// Convert realsense2 point cloud pts to pcl
pcl_ptr points_to_pcl(const rs2::points& points);