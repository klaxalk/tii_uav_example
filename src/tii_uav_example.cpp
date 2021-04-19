/* includes //{ */

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>

#include <nav_msgs/Odometry.h>

#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/Path.h>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/mutex.h>

//}

/* state machine //{ */

typedef enum
{
  IDLE,
  GOTO,
  ENCIRCLING
} State_t;

const char* state_names[3] = {"IDLE_STATE", "GOTO_STATE", "ENCIRCLING_STATE"};

//}

namespace tii_uav_example
{

/* class TiiUavExample //{ */

class TiiUavExample : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_ = false;

  // | ------------------- subscribe handlers ------------------- |

  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_other_uav_odom_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_this_uav_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;

  void callbackOtherUavOdom(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& sh);

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<mrs_msgs::PathSrv> sch_path_;

  // | ---------------------- state machine --------------------- |

  void       changeState(State_t new_state);
  State_t    current_state_;
  std::mutex mutex_state_;

  // | --------------------- service server --------------------- |

  ros::ServiceServer service_server_;
};

//}

/* onInit() //{ */

void TiiUavExample::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  // waiting for current time to be available
  ros::Time::waitForValid();

  // | ------------------- subscribe handlers ------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "TiiUavExample";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_other_uav_odom_       = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "other_uav_odom_in", &TiiUavExample::callbackOtherUavOdom, this);
  sh_this_uav_cmd_         = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "this_uav_cmd_in");
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "this_uav_control_diag_in");

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(1.0), &TiiUavExample::timerMain, this);

  // | --------------------- service clients -------------------- |

  sch_path_ = mrs_lib::ServiceClientHandler<mrs_msgs::PathSrv>(nh_, "path_out");

  // | --------------------- service servers -------------------- |

  /* service_server_  = nh_.advertiseService("service_in", &TiiUavExample::callbackStart, this); */

  is_initialized_ = true;

  ROS_INFO("[TiiUavExample]: initialized, noted");
}

//}

/* callbackOtherUavOdom() //{ */

void TiiUavExample::callbackOtherUavOdom(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& sh) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[TiiUavExample]: getting other UAV odometry");
}

//}

/* timerMain() //{ */

void TiiUavExample::timerMain(const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[TiiUavExample]: timerMain is spinning");

  if (!sh_this_uav_cmd_.hasMsg()) {
    ROS_INFO("[TiiUavExample]: waiting for this UAV cmd");
    return;
  }

  if (!sh_other_uav_odom_.hasMsg()) {
    ROS_INFO("[TiiUavExample]: waiting for other UAV odom");
    return;
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    ROS_INFO("[TiiUavExample]: waiting for control manager diag");
    return;
  }

  ROS_INFO_ONCE("[TiiUavExample]: we got all the data we need");

  changeState(GOTO);

  auto current_state = mrs_lib::get_mutexed(mutex_state_, current_state_);

  auto other_drone_odom = sh_other_uav_odom_.getMsg();
  auto this_drone_cmd   = sh_this_uav_cmd_.getMsg();

  switch (current_state) {

    case IDLE: {

      ROS_INFO_THROTTLE(1.0, "[TiiUavExample]: idling");

      break;
    }

    case GOTO: {

      ROS_INFO_THROTTLE(1.0, "[TiiUavExample]: goto state");

      mrs_msgs::PathSrv srv_out;

      srv_out.request.path.fly_now     = true;
      srv_out.request.path.use_heading = true;

      mrs_msgs::Reference new_ref;
      new_ref.position.x = other_drone_odom->pose.pose.position.x;
      new_ref.position.y = other_drone_odom->pose.pose.position.y;
      new_ref.position.z = other_drone_odom->pose.pose.position.z;

      const double des_heading = atan2(new_ref.position.y - this_drone_cmd->pose.pose.position.y, new_ref.position.x - this_drone_cmd->pose.pose.position.x);

      new_ref.heading = des_heading;

      srv_out.request.path.points.push_back(new_ref);

      sch_path_.call(srv_out);

      break;
    }

    case ENCIRCLING: {

      ROS_INFO_THROTTLE(1.0, "[TiiUavExample]: encircling");

      break;
    }
  }
}

//}

/* changeState() //{ */

void TiiUavExample::changeState(State_t new_state) {

  {
    std::scoped_lock lock(mutex_state_);

    State_t previous_state = current_state_;
    current_state_         = new_state;
  }

  switch (new_state) {
    case IDLE: {

      break;
    }

    case GOTO: {

      break;
    }

    case ENCIRCLING: {

      break;
    }
  }

  /* ROS_INFO("[TiiUavExample]: change state form %s -> %s", state_names[previous_state], state_names[new_state]); */
}

//}

}  // namespace tii_uav_example

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tii_uav_example::TiiUavExample, nodelet::Nodelet)
