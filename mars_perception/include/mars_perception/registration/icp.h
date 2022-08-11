#pragma once

#include <pcl/registration/icp.h>
#include <mars_perception/registration/registration_base.h>

class ICP : public RegBase
{
public:
    static const std::string NAME;

    ICP() : max_corresp_dist_(1e-2), transf_epsil_(1e-11), fitness_epsil_(1), max_iter_(20), RegBase() {};
    void run();

private:
    double max_corresp_dist_; 
    double transf_epsil_;
    double fitness_epsil_;
    double max_iter_;
};