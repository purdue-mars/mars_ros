#include <mars_perception/registration/icp.h>

const std::string ICP::NAME = "ICP";

void ICP::run()
{
    try
    {
        if (scene_ptr->empty() || mesh_ptr->empty())
        {
            return;
        }
        pcl::IterativeClosestPoint<Point, Point> icp;
        icp.setInputSource(mesh_ptr);
        icp.setInputTarget(scene_ptr);
        // pcl::registration::WarpPoint3dTransRot<Point, Point>::Ptr warp_fcn(new pcl::registration::WarpPoint3dTransRot<Point, Point>);
        // pcl::registration::TransformationEstimationLM<Point, Point>::Ptr te(new pcl::registration::TransformationEstimationLM<Point, Point>);
        // te->setWarpFunction(warp_fcn);
        // icp.setTransformationEstimation(te);
        icp.align(*mesh_ptr);

        *tf_mat_ptr = icp.getFinalTransformation() * *tf_mat_ptr;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}