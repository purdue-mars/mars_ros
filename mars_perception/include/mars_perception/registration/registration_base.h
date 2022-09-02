#pragma once

#include <mars_perception/common.h>

typedef std::vector<std::string> RegistrationCfg;

class RegBase
{
public:
    RegBase() : scene_ptr(new PointCloud), mesh_ptr(new PointCloud), tf_mat_ptr(std::make_shared<TFMatrix>(TFMatrix::Identity())) {}
    PointCloudPtr scene_ptr;
    PointCloudPtr mesh_ptr;
    std::shared_ptr<TFMatrix> tf_mat_ptr;
};