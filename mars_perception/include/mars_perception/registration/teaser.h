#pragma once

#include <teaser/registration.h> 
#include <mars_perception/registration/registration_base.h>

class Teaser : public RegBase
{
public:
    static const std::string NAME;
    Teaser() : solver(params), RegBase() {};
    Teaser(PointCloudPtr scene, PointCloudPtr mesh): solver(params), RegBase(scene,mesh) {}
    void run();

private:
  teaser::RobustRegistrationSolver::Params params;
  teaser::RobustRegistrationSolver solver;
};
