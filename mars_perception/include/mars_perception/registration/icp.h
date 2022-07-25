#pragma once

#include <pcl/registration/icp.h>
#include <mars_perception/registration/registration_base.h>

#define ICP_NAME "ICP"

class ICP : public RegBase
{
public:

    ICP() : max_corresp_dist_(1e-2), transf_epsil_(1e-11), fitness_epsil_(1), max_iter_(20), RegBase() {};
    ICP(double max_corresp_dist, double transf_epsil, double fitness_epsil, double max_iter, PointCloudPtr scene, PointCloudPtr mesh): 
        max_iter_(max_iter), 
        max_corresp_dist_(max_corresp_dist), 
        transf_epsil_(transf_epsil), 
        fitness_epsil_(fitness_epsil), RegBase(scene,mesh) {}
    void run();

private:
    double max_corresp_dist_; 
    double transf_epsil_;
    double fitness_epsil_;
    double max_iter_;
};