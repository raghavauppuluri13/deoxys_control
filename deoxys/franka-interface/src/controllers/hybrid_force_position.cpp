// Copyright 2023 Raghava

#include "franka_controller.pb.h"
#include "franka_robot_state.pb.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "utils/common_utils.h"
#include "utils/control_utils.h"
#include "utils/robot_utils.h"

#include "controllers/hybrid_force_position.h"

#include <memory>

namespace controller {
HybridForcePositionController::HybridForcePositionController() {}
HybridForcePositionController::~HybridForcePositionController() {}

HybridForcePositionController::HybridForcePositionController(
    franka::Model &model) {
  model_ = &model;
}

bool HybridForcePositionController::ParseMessage(
    const FrankaControlMessage &msg) {

  if (!msg.control_msg().UnpackTo(&control_msg_)) {
    return false;
  }

  control_msg_.translational_stiffness();

  cartesian_stiffness_.setIdentity();
  cartesian_stiffness_.topLeftCorner<3, 3>()
      << control_msg_.translational_stiffness() * Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner<3, 3>()
      << control_msg_.rotational_stiffness() * Eigen::Matrix3d::Identity();
  cartesian_stiffness_(2, 2) = 0.0;

  cartesian_damping_.setIdentity();
  cartesian_damping_.topLeftCorner<3, 3>()
      << 2.0 * std::sqrt(control_msg_.translational_stiffness()) *
             Eigen::Matrix3d::Identity();
  cartesian_damping_.bottomRightCorner<3, 3>()
      << 2.0 * std::sqrt(control_msg_.rotational_stiffness()) *
             Eigen::Matrix3d::Identity();

  cartesian_damping_(2, 2) = 0.0;

  kp_ = control_msg_.kp();
  ki_ = control_msg_.ki();

  this->state_estimator_ptr_->ParseMessage(msg.state_estimator_msg());
  return true;
}

void HybridForcePositionController::ComputeGoal(
    const std::shared_ptr<StateInfo> &current_state_info,
    std::shared_ptr<StateInfo> &goal_state_info) {
  if (control_msg_.goal().is_delta()) {
    goal_state_info->pos_EE_in_base_frame =
        current_state_info->pos_EE_in_base_frame +
        Eigen::Vector3d(control_msg_.goal().x(), control_msg_.goal().y(),
                        control_msg_.goal().z());
    Eigen::AngleAxisd relative_axis_angle;
    Eigen::Vector3d relative_ori(control_msg_.goal().ax(),
                                 control_msg_.goal().ay(),
                                 control_msg_.goal().az());
    AxisAngle(relative_ori, relative_axis_angle);
    goal_state_info->quat_EE_in_base_frame =
        Eigen::Quaterniond(relative_axis_angle.toRotationMatrix() *
                           current_state_info->quat_EE_in_base_frame);

  } else {
    goal_state_info->pos_EE_in_base_frame =
        current_state_info->pos_EE_in_base_frame +
        Eigen::Vector3d(control_msg_.goal().x(), control_msg_.goal().y(),
                        control_msg_.goal().z());
    Eigen::AngleAxisd absolute_axis_angle;
    Eigen::Vector3d absolute_ori(control_msg_.goal().ax(),
                                 control_msg_.goal().ay(),
                                 control_msg_.goal().az());
    AxisAngle(absolute_ori, absolute_axis_angle);
    goal_state_info->quat_EE_in_base_frame =
        Eigen::Quaterniond(absolute_axis_angle);
  }

  goal_state_info->wrench_in_sensor_frame =
      Eigen::Vector3d(control_msg_.goal().fx(), control_msg_.goal().fy(),
                      control_msg_.goal().fz());
  // goal_state_info->joint_positions << control_msg_.goal().q1(),
  // control_msg_.goal().q2(), control_msg_.goal().q3(),
  // control_msg_.goal().q4(), control_msg_.goal().q5(),
  // control_msg_.goal().q6(), control_msg_.goal().q7();
}

std::array<double, 7> HybridForcePositionController::Step(
    const franka::RobotState &robot_state,
    const Eigen::Vector3d &wrench_in_sensor_frame,
    const Eigen::Vector3d &desired_wrench_in_sensor_frame,
    const Eigen::Vector3d &desired_pos_EE_in_base_frame,
    const Eigen::Quaterniond &desired_quat_EE_in_base_frame) {

  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();

  Eigen::Matrix<double, 6, 1> force_ext(
      Eigen::Matrix<double, 6, 1>::Map(robot_state.O_F_ext_hat_K.data()));

  if (!initialized_) {
    force_ext_initial_ = force_ext;
    initialized_ = true;
  }

  Eigen::Matrix<double, 7, 1> tau_d;

  std::array<double, 49> mass_array = model_->mass(robot_state);
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data());

  // coriolis and gravity
  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  // jacobian
  std::array<double, 42> jacobian_array =
      model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  // transform
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // Current joint velocity
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  // FORCE CONTROL
  Eigen::Matrix<double, 6, 1> desired_force_torque, force_control;
  Eigen::Matrix<double, 7, 1> tau_force, tau_task, tau_cartesian_impedance,
      tau_cmd;

  desired_force_torque.setZero();
  // desired_force_torque(2) = -desired_force_;
  desired_force_torque.head<3>() = desired_wrench_in_sensor_frame;

  force_error_ =
      force_error_ + (desired_force_torque - force_ext + force_ext_initial_);
  force_control =
      (desired_force_torque +
       kp_ * (desired_force_torque - force_ext + force_ext_initial_) +
       ki_ * force_error_);
  force_control << force_control(0), force_control(1), force_control(2), 0, 0,
      0;
  tau_force = jacobian.transpose() * force_control;
  tau_d << tau_force;

  std::array<double, 7> tau_d_array{};
  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

  return tau_d_array;
}
} // namespace controller
