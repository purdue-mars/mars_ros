#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <mars_msgs/RegistrationSrv.h>
#include <mars_perception/common.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <mars_perception/registration/registration_base.h>
#include <mars_perception/registration/icp.h>
#include <mars_perception/registration/constrained_icp.h>
#include <dynamic_reconfigure/server.h>
#include <mars_config/RegistrationConfig.h>

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
    std::shared_ptr<TFMatrix> tf_mat_ptr_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster br_;
    ICP icp_;
    ConstrainedICP constr_icp_;

    std::string alg_;
    std::string base_frame_;

    dynamic_reconfigure::Server<mars_config::RegistrationConfig> param_server_;
    dynamic_reconfigure::Server<mars_config::RegistrationConfig>::CallbackType f_;

    bool initialized_; 

    bool registration_srv_cb_(mars_msgs::RegistrationSrv::Request &req, mars_msgs::RegistrationSrv::Response &resp);
    void scene_pc_cb_(const PointCloudMsg::ConstPtr &msg);
    void update_params_cb_(mars_config::RegistrationConfig &config, uint32_t level);
};

RegistrationServer::RegistrationServer() : scene_pc_(new PointCloud), tf_mat_ptr_(std::make_shared<TFMatrix>(TFMatrix::Identity())), initialized_(false)
{    
    ros::param::get("base_link", base_frame_);
    ros::param::get("~alg", alg_);

    std::string scene_pc_topic;
    ros::param::get("~filtered_points_topic", scene_pc_topic);
    registration_server_ = n_.advertiseService("registration", &RegistrationServer::registration_srv_cb_, this);
    scene_pc_sub_ = n_.subscribe(scene_pc_topic, 10, &RegistrationServer::scene_pc_cb_, this);
    mesh_pub_ = n_.advertise<sensor_msgs::PointCloud2>("object_mesh_pc", 10);

    f_ = boost::bind(&RegistrationServer::update_params_cb_, this, _1, _2);
    param_server_.setCallback(f_);

    icp_.scene_ptr = scene_pc_; 
    icp_.mesh_ptr = mesh_.get_pc_ptr(); 
    icp_.tf_mat_ptr = tf_mat_ptr_;
    constr_icp_.scene_ptr = scene_pc_; 
    constr_icp_.mesh_ptr = mesh_.get_pc_ptr(); 
    constr_icp_.tf_mat_ptr = tf_mat_ptr_;
}

void RegistrationServer::scene_pc_cb_(const PointCloudMsg::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, *scene_pc_);
}

bool RegistrationServer::registration_srv_cb_(mars_msgs::RegistrationSrv::Request &req, mars_msgs::RegistrationSrv::Response &resp) {
    icp_.reset();
    Eigen::Affine3d mat;
    mat.translation() << req.init_tf.translation.x,req.init_tf.translation.y, req.init_tf.translation.z;
    mat.rotate(Eigen::Quaterniond(req.init_tf.rotation.x,
                                  req.init_tf.rotation.y, 
                                  req.init_tf.rotation.z,
                                  req.init_tf.rotation.w));
    bool result = mesh_.update_mesh(req.mesh_name,mat);
    *tf_mat_ptr_ = mat.matrix().cast<float>();
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
        Eigen::Quaternionf q(tf_mat_ptr_->topLeftCorner<3, 3>());
        transform.setOrigin(tf::Vector3(tf_mat_ptr_->col(3)(0), tf_mat_ptr_->col(3)(1), tf_mat_ptr_->col(3)(2)));
        transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
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

        // Publish Mesh
        PointCloudPtr mesh_ptr = mesh_.get_pc_ptr();
        mesh_ptr->header.frame_id = base_frame_;
        mesh_pub_.publish(mesh_ptr);
    }
}

void RegistrationServer::update_params_cb_(mars_config::RegistrationConfig &config, uint32_t level) {
    constr_icp_.set_axes(
        config.groups.constr_icp.rot_ax_i, 
        config.groups.constr_icp.trans_ax0_i,
        config.groups.constr_icp.trans_ax1_i
    );
    ROS_INFO("Reconfigure Request: %d %d %d", 
        config.groups.constr_icp.rot_ax_i, 
        config.groups.constr_icp.trans_ax0_i,
        config.groups.constr_icp.trans_ax1_i);
}

void RegistrationServer::run() {
    if(alg_ == icp_.NAME) {
        icp_.run();
    }
    else if(alg_ == constr_icp_.NAME) {
        constr_icp_.run();
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