#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <mars_perception/common.h>
#include <pcl/point_types.h>

class PCWriter {

  public:
    PCWriter();
    void pc_cb_(const PointCloudMsg::ConstPtr &msg);
    void write();
  private:
    ros::NodeHandle n_;
    PointCloudPtr p_;
    ros::Subscriber sub_;
};

PCWriter::PCWriter() : p_(new PointCloud) {
  std::string topic;
  ros::param::get("~topic", topic);
  sub_ = n_.subscribe(topic,1,&PCWriter::pc_cb_,this);
}

void PCWriter::pc_cb_(const PointCloudMsg::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, *p_);
}

void PCWriter::write() {
  if(!p_->empty()) {
    pcl::io::savePCDFileASCII("/home/ruppulur/catkin_ws/src/icra_2022_mars/mars_perception/src/out.pcd", *p_);
  }
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "pcd_write");
  PCWriter p;

  ros::Rate r(1);
  while(ros::ok()) {
    ros::spinOnce();
    p.write();
    r.sleep();
  }
  return (0);
}
