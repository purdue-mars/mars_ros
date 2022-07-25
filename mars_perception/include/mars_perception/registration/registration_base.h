#pragma once

#include <mars_perception/common.h>

typedef std::vector<std::string> RegistrationCfg;

class RegBase
{
public:
    RegBase() : scene_ptr(new PointCloud), mesh_ptr(new PointCloud), tf_mat(TFMatrix::Identity()) {}
    RegBase(PointCloudPtr scene, PointCloudPtr mesh) : scene_ptr(scene), mesh_ptr(mesh), tf_mat(TFMatrix::Identity()) {}

    PointCloudPtr scene_ptr;
    PointCloudPtr mesh_ptr;
    TFMatrix tf_mat;

    void reset() {
        tf_mat = TFMatrix::Identity();
    }
};