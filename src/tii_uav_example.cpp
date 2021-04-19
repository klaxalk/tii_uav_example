#include <ros/ros.h>
#include <nodelet/nodelet.h>

namespace tii_uav_example
{

class TiiUavExample : public nodelet::Nodelet {

public:
  virtual void onInit();
};

void TiiUavExample::onInit() {

  ROS_INFO("[TiiUavExample]: initialized");
}

}  // namespace tii_uav_example

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tii_uav_example::TiiUavExample, nodelet::Nodelet)
