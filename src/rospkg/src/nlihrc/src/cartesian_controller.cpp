// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <nlihrc/cartesian_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>


#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace nlihrc {

bool CartesianController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  
  sub_command_ = node_handle.subscribe(
      "command", 1, &CartesianController::command_callback_, this,
      ros::TransportHints().reliable().tcpNoDelay());
  
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianController: Could not get state interface from hardware");
    return false;
  }

  return true;
}

void CartesianController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.001);
  state_delta_ = 0.0;
  linear_state_offset_x_ = 0.0;
  linear_state_offset_y_ = 0.0;
  state_ = IDLE;
  buffer_available_ = false;
  // Defaults aren't used anywhere but initialized for sanity
  inp_axis_ = ZAxis;
  inp_direction_positive_ = false;
  inp_linear_runtime_ = 1.0;
  buffer_distance_ = 0.0;
  state_previous_step_ = 0;

}

void CartesianController::load_start_state_(double xsec)
{
  double current_step = state_previous_step_;

  // ROS_INFO_STREAM("ELAPSED TIME: " << elapsed_time_.toSec());
  // ROS_INFO_STREAM("Current step: " << current_step);

  if (xsec <= smooth_c_)
  {
    current_step = (smooth_b_*exp(smooth_r_*xsec)/(exp(smooth_c_*smooth_r_)+exp(smooth_r_*xsec)));
    state_ = START;
    // ROS_DEBUG_STREAM("RUNNING START STATE");
  }
  else
  {
    // HARDCODED!
    current_step = 0.25*smooth_b_*smooth_r_*(xsec-0.32);

    linear_state_offset_y_ = current_step-0.5*smooth_b_;
    linear_state_offset_x_ = xsec;
    state_ = LINEAR;
    // ROS_DEBUG_STREAM("RUNNING LINEAR STATE");
  }

  // ROS_INFO_STREAM("Current Step: " << current_step);
  state_delta_ = current_step-state_previous_step_;
  if (state_previous_step_ == 0)
  {
    state_delta_ = 0;
  }
  state_previous_step_ = current_step;
}

void CartesianController::load_stop_state_(double xsec)
{
  double current_step = state_previous_step_;
  state_ = STOP;
  // ROS_DEBUG_STREAM("Linear state offset x: " << linear_state_offset_x_);

  if (xsec>linear_state_offset_x_ and xsec <= (linear_state_offset_x_+smooth_c_))
  {
    // ROS_DEBUG_STREAM("RUNNING STOP STATE");
    double z = xsec-linear_state_offset_x_;
    current_step = (smooth_b_*exp(smooth_r_*z)/(1+exp(smooth_r_*z))) + linear_state_offset_y_;
    state_delta_ = current_step-state_previous_step_;
  }
  else if (xsec > (linear_state_offset_x_+1.15*smooth_c_))
  {
    state_ = IDLE;
    state_delta_ = 0;
  }
  state_previous_step_ = current_step;

}

void CartesianController::update_pose_(std::array<double, 16>& pose, const Axis axis, const bool direction)
{
  pose[axis+12] = pose[axis+12] + (2*direction - 1)*state_delta_;
}

bool CartesianController::limits_valid_(const std::array<double, 16>& pose, const Axis axis, const bool is_positive)
{
  const std::array<const std::array<double, 2>,3> axis_limits = {x_limits_, y_limits_, z_limits_};
  
  // ROS_INFO_STREAM("Pose: " << pose[axis+12]);
  // ROS_INFO_STREAM("Direction: " << is_positive);

  if (is_positive and (axis_limits[axis][is_positive]-limit_offset_) < pose[axis+12])
  {
    ROS_WARN_STREAM_THROTTLE(5, "Cartesian limit reached at axis (0-X, 1-Y, 2-Z) index: " << static_cast<int>(axis)
    << " in positive direction. Try moving in the opposite direction.");
    return false;
  }
  else if (not is_positive and (axis_limits[axis][is_positive]+limit_offset_) > pose[axis+12])
  {
    ROS_WARN_STREAM_THROTTLE(5, "Cartesian limit reached at axis (0-X, 1-Y, 2-Z) index: " << static_cast<int>(axis)
    << " in negative direction. Try moving in the opposite direction.");
    return false;
  }

  return true;
}

void CartesianController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {

  std::array<double, 16> desired_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  const double xsec=elapsed_time_.toSec();


  if (state_ != IDLE)
  {

  // ROS_INFO_STREAM("Elapsed Seconds: " << xsec);
    if (state_ == START or state_ == LINEAR)
    {  
      // ROS_INFO_STREAM("State delta: " << state_delta_);

      load_start_state_(xsec);
      if (state_ == LINEAR)
      {
        // Test
        if (not limits_valid_(desired_pose, inp_axis_, inp_direction_positive_) or xsec > inp_linear_runtime_)
        {
          load_stop_state_(xsec);
        }
      }
    }
    else if (state_ == STOP)
    {
      load_stop_state_(xsec);
    }

    update_pose_(desired_pose, inp_axis_, inp_direction_positive_);

    //Hardcoded update frequency to cater instability in update function call period
    elapsed_time_ += ros::Duration(0.001);
  }
  else
  {
    // get queued data
    if (buffer_available_)
    {
      inp_axis_ = buffer_axis_;
      inp_direction_positive_ = buffer_direction_positive_;
      inp_linear_runtime_ = buffer_linear_runtime_;
      elapsed_time_ = ros::Duration(0.001);
      state_previous_step_ = 0;

      ROS_INFO_STREAM("Initializing new task. Axis: " 
      << static_cast<int>(inp_axis_) <<
      " Direction: " << inp_direction_positive_
       << " Linear Runtime: "
        << inp_linear_runtime_);
      
      state_ = START;
      buffer_available_ = false;
    }
  }
  
  cartesian_pose_handle_->setCommand(desired_pose);

}

void CartesianController::command_callback_(const std_msgs::String& msg)
{
  if (state_ == START or state_ == STOP)
  {
    return;
  }
  // Get msg keys
  std::string data = msg.data;
  char delimiter = ',';
  size_t pos;
  std::istringstream ss(data);
  std::string item;
  std::vector<std::string> elems;
  while (std::getline(ss,item,delimiter)) {
    elems.push_back(item);
    }
  if (elems.size() != 2)
  {
    return;
  }
  // Get axis, direction from message
  const Axis axis = static_cast<Axis>((int)elems[0][0]-88);
  const double distance = atof(elems[1].c_str());

  if (state_ == LINEAR)
  {
    state_ = STOP;
  }
  // if distance is zero, initialize stop state without filling buffer
  if (distance == 0)
  {
    return;
  }
  
  buffer_direction_positive_ = distance > 0;
  // HARDCODED! From linear equation:
  //    distance = 0.25*smooth_b_*smooth_r_*(time-0.32);
  buffer_linear_runtime_ = std::abs(distance-2*smooth_b_)/(0.25*smooth_b_*smooth_r_) + 0.32;
  buffer_axis_ = axis;
  buffer_available_ = true;
  buffer_distance_ = distance;

}

}  // namespace end

PLUGINLIB_EXPORT_CLASS(nlihrc::CartesianController,
                       controller_interface::ControllerBase)
