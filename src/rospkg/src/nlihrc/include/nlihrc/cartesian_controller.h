#pragma once

#include <array>
#include <memory>
#include <string>
#include <sstream>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/String.h>

#include <franka_hw/franka_cartesian_command_interface.h>

namespace nlihrc {

enum State {START, LINEAR, STOP, IDLE};
enum Axis {XAxis=0, YAxis=1, ZAxis=2};

class CartesianController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  ros::Duration elapsed_time_;
  //limits
  const std::array<double, 2> x_limits_={0.3, 0.6};
  const std::array<double, 2> y_limits_={-0.25, 0.25};
  const std::array<double, 2> z_limits_={0.05, 0.5};
  const double limit_offset_ = 0.025;
  // state variables
  double linear_state_offset_x_;
  double linear_state_offset_y_;
  double state_delta_;
  double state_previous_step_;
  const double smooth_c_ = 0.4;
  const double smooth_b_ = 0.04;
  const double smooth_r_ = 25; 
  State state_;
  // pose
  std::array<double, 16> initial_pose_{};
  // inputs and buffer
  bool inp_direction_positive_;
  double inp_linear_runtime_;
  Axis inp_axis_;
  bool buffer_available_;
  bool buffer_direction_positive_;
  double buffer_distance_;
  double buffer_linear_runtime_;
  Axis buffer_axis_;
  // methods
  void load_start_state_(double xsec);
  void load_stop_state_(double xsec);
  void update_pose_(std::array<double, 16>& pose, const Axis axis, const bool direction);
  bool limits_valid_(const std::array<double, 16>& pose, const Axis axis, const bool is_positive);

  ros::Subscriber sub_command_;
  void command_callback_(const std_msgs::String& msg);
};

}  // namespace end
