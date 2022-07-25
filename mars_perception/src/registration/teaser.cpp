#include <mars_perception/registration/teaser.h>

void Teaser::run()
{
    try
    {
        if (scene_ptr->empty() || mesh_ptr->empty())
        {
            return;
        }

        Eigen::Matrix<double, 3, Eigen::Dynamic> scene(3, scene_ptr->size());
        for (size_t i = 0; i < scene_ptr->size(); ++i) {
            scene.col(i) << scene_ptr->points[i].x, scene_ptr->points[i].y, scene_ptr->points[i].z;
        }
        Eigen::Matrix<double, 3, Eigen::Dynamic> mesh(3, mesh_ptr->size());
        for (size_t i = 0; i < mesh_ptr->size(); ++i) {
            mesh.col(i) << mesh_ptr->points[i].x, mesh_ptr->points[i].y, mesh_ptr->points[i].z;
        }
        solver.solve(mesh, scene);
        auto solution = solver.getSolution();

        std::cerr << "TEASER: INVALID SOLUTION!" << '\n';
        if(solution.valid) {
            std::cerr << "TEASER: INVALID SOLUTION!" << '\n';

            tf_mat.topLeftCorner<3,3>() = solution.rotation.cast<float>();
            tf_mat.topRightCorner<3,1>() = solution.translation.cast<float>();
        }
        else {
            std::cerr << "TEASER: INVALID SOLUTION!" << '\n';
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}