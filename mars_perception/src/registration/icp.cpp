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
        icp.align(*mesh_ptr);

        *tf_mat_ptr = icp.getFinalTransformation() * *tf_mat_ptr;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}