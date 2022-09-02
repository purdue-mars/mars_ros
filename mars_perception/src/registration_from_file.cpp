#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <mars_msgs/RegistrationSrv.h>
#include <mars_perception/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <mars_perception/registration/registration_base.h>
#include <mars_perception/registration/icp.h>
//#include <mars_perception/registration/constrained_icp.h>
#include <dynamic_reconfigure/server.h>
#include <mars_config/RegistrationConfig.h>

class RegistrationFromFile
{
public:
    RegistrationFromFile();
    void run();
    void publish();

private:
    ros::NodeHandle n_;
    PointCloudPtr scene_pc_;
    PointCloudPtr mesh_pc_;
    ros::Publisher mesh_pub_;
    ros::Publisher scene_pub_;
    std::shared_ptr<TFMatrix> tf_mat_ptr_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster br_;
    ICP icp_;
    //ConstrainedICP constr_icp_;

    std::string alg_;
    std::string base_frame_;
    bool initialized_;
    bool running_;
};

RegistrationFromFile::RegistrationFromFile() : scene_pc_(new PointCloud), mesh_pc_(new PointCloud), tf_mat_ptr_(std::make_shared<TFMatrix>(TFMatrix::Identity())), initialized_(false)
{
    std::string data_folder;
    ros::param::param<std::string>("~data_folder", data_folder, "/home/ruppulur/catkin_ws/src/icra_2022_mars/mars_perception/src/data");
    std::vector<double> init_mesh_pose;
    ros::param::param<std::vector<double>>("~init_mesh_pose", init_mesh_pose, {0, 0.008, -0.015, 0, 0, 0.7071068, 0.7071068});
    if (pcl::io::loadPCDFile<Point>(data_folder + "/pcd_0.pcd", *scene_pc_) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file! \n");
        exit(0);
    }
    if (pcl::io::loadPCDFile<Point>(data_folder + "/pcd_1.pcd", *mesh_pc_) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file! \n");
        exit(0);
    }

    mesh_pub_ = n_.advertise<sensor_msgs::PointCloud2>("mesh_pc", 10);
    scene_pub_ = n_.advertise<sensor_msgs::PointCloud2>("scene_pc", 10);

    ros::param::param<std::string>("~base_link", base_frame_, "base_link");
    ros::param::param<std::string>("~alg", alg_, "ICP");
    ros::param::param<bool>("~running", running_, true);
    icp_.scene_ptr = scene_pc_;
    icp_.mesh_ptr = mesh_pc_;
    icp_.tf_mat_ptr = tf_mat_ptr_;
    // constr_icp_.scene_ptr = scene_pc_;
    // constr_icp_.mesh_ptr = mesh_pc_;
    // constr_icp_.tf_mat_ptr = tf_mat_ptr_;

    std::vector<double> tf(init_mesh_pose);
    assert(tf.size() == 7);
    *tf_mat_ptr_ = TFMatrix::Identity();
    Eigen::Affine3d mat(Eigen::Affine3d::Identity());
    mat.translation() << tf[0], tf[1], tf[2];
    Eigen::Quaterniond q(tf[3], tf[4], tf[5], tf[6]);
    q.normalize();
    mat.rotate(q);
    pcl::transformPointCloud(*scene_pc_, *scene_pc_, mat.matrix().cast<float>());
    // *tf_mat_ptr_ = mat.matrix().cast<float>();
    initialized_ = true;
}

void RegistrationFromFile::publish()
{
    if (initialized_)
    {
        tf::Transform transform;
        std::string frame_id = "mesh";
        geometry_msgs::TransformStamped tf;
        Eigen::Quaternionf q(tf_mat_ptr_->topLeftCorner<3, 3>());
        transform.setOrigin(tf::Vector3(tf_mat_ptr_->col(3)(0), tf_mat_ptr_->col(3)(1), tf_mat_ptr_->col(3)(2)));
        transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()).normalize());
        tf.child_frame_id = frame_id;
        tf.header.frame_id = base_frame_;
        tf.header.stamp = ros::Time::now();
        tf.transform.translation.x = tf_mat_ptr_->col(3)(0);
        tf.transform.translation.y = tf_mat_ptr_->col(3)(1);
        tf.transform.translation.z = tf_mat_ptr_->col(3)(2);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        br_.sendTransform(tf);

        // Publish
        mesh_pc_->header.frame_id = base_frame_;
        mesh_pub_.publish(mesh_pc_);

        scene_pc_->header.frame_id = base_frame_;
        scene_pub_.publish(scene_pc_);
    }
}

void RegistrationFromFile::run()
{
    if(!running_) return;
    if (alg_ == icp_.NAME)
    {
        icp_.run();
    }
    // else if (alg_ == constr_icp_.NAME)
    // {
    //     constr_icp_.run();
    // }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "registration_from_file");
    RegistrationFromFile srv;
    ros::Rate r(5);
    while (ros::ok())
    {
        ros::spinOnce();
        srv.publish();
        ROS_INFO("Published!");
        srv.run();
        ROS_INFO("!");
        r.sleep();
    }
}