#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <mars_msgs/RegistrationSrv.h>
#include <mars_perception/common.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <mars_perception/registration/registration_base.h>
#include <mars_perception/registration/icp.h>

class RegistrationServer
{
public:
    RegistrationServer();
    void run();
    void publish();
private:
    ros::NodeHandle n_;
    MeshUtil mesh_;
    ros::ServiceServer registration_server_;
    ros::Publisher mesh_pub_;
    ros::Subscriber scene_pc_sub_;

    PointCloudPtr scene_pc_;
    TFMatrix tf_mat_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster br_;
    ICP icp_;

    std::string alg_;
    std::string base_frame_;

    bool initialized_; 

    bool registration_srv_cb_(mars_msgs::RegistrationSrv::Request &req, mars_msgs::RegistrationSrv::Response &resp);
    void scene_pc_cb_(const PointCloudMsg::ConstPtr &msg);
};

RegistrationServer::RegistrationServer() : scene_pc_(new PointCloud), tf_mat_(TFMatrix::Identity()), initialized_(false)
{    
    ros::param::get("base_link", base_frame_);
    ros::param::get("~alg", alg_);

    std::string scene_pc_topic;
    ros::param::get("~filtered_points_topic", scene_pc_topic);
    registration_server_ = n_.advertiseService("registration", &RegistrationServer::registration_srv_cb_, this);
    scene_pc_sub_ = n_.subscribe(scene_pc_topic, 10, &RegistrationServer::scene_pc_cb_, this);
    mesh_pub_ = n_.advertise<sensor_msgs::PointCloud2>("object_mesh_pc", 10);

    double max_corresp_dist, transf_epsil, fitness_epsil, max_iter;
    ros::param::param<double>("~max_correspondence_distance", max_corresp_dist, 0.5);
    ros::param::param<double>("~transformation_epsilon", transf_epsil, 1e-11);
    ros::param::param<double>("~fitness_epsilon", fitness_epsil, 1e-3);
    ros::param::param<double>("~max_iterations", max_iter, 100);

    icp_ = ICP(max_corresp_dist, transf_epsil, fitness_epsil, max_iter, scene_pc_, mesh_.get_pc_ptr());
}

void RegistrationServer::scene_pc_cb_(const PointCloudMsg::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, *scene_pc_);
}

bool RegistrationServer::registration_srv_cb_(mars_msgs::RegistrationSrv::Request &req, mars_msgs::RegistrationSrv::Response &resp) {
    icp_.reset();
    bool result = mesh_.update_mesh(req.mesh_name);
    if (!result)
    {
        ROS_ERROR("Update mesh failed! Double check that mesh .obj file is valid!");
        return false;
    }
    initialized_ = true;
    return true;
}

void RegistrationServer::publish() {
    if(initialized_) {
        tf::Transform transform;
        std::string frame_id = mesh_.get_name();
        geometry_msgs::TransformStamped tf;
        Eigen::Quaternionf q(tf_mat_.topLeftCorner<3, 3>());
        transform.setOrigin(tf::Vector3(tf_mat_.col(3)(0), tf_mat_.col(3)(1), tf_mat_.col(3)(2)));
        transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        tf.child_frame_id = frame_id;
        tf.header.frame_id = base_frame_;
        tf.header.stamp = ros::Time::now();
        tf.transform.translation.x = tf_mat_.col(3)(0);
        tf.transform.translation.y = tf_mat_.col(3)(1);
        tf.transform.translation.z = tf_mat_.col(3)(2);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        br_.sendTransform(tf);

        // Publish Mesh
        PointCloudPtr mesh_ptr = mesh_.get_pc_ptr();
        mesh_ptr->header.frame_id = base_frame_;
        mesh_pub_.publish(mesh_ptr);
    }
}

void RegistrationServer::run() {
    if(alg_ == ICP_NAME) {
        icp_.run();
        tf_mat_ = icp_.tf_mat;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "registration_server");
    RegistrationServer srv;
    ros::Rate r(50);
    while (ros::ok())
    {
        ros::spinOnce();
        srv.run();
        srv.publish();
        r.sleep();
    }
}