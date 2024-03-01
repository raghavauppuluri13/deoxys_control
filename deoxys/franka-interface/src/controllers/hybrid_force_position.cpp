// Copyright 2023 Raghava

#include "franka_controller.pb.h"
#include "franka_robot_state.pb.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/SVD>
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

  double translational_stiffness = 1500.0;
  double rotational_stiffness = 100.0;
  double translational_damping = 0.5;
  double rotational_damping = 10.0;

  cartesian_stiffness_.setIdentity();
  cartesian_stiffness_.topLeftCorner<3, 3>()
      << translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner<3, 3>()
      << rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_.setIdentity();
  cartesian_damping_.topLeftCorner<3, 3>()
      << 2.0 * std::sqrt(translational_damping) * Eigen::Matrix3d::Identity();
  cartesian_damping_.bottomRightCorner<3, 3>()
      << 2.0 * std::sqrt(rotational_damping) * Eigen::Matrix3d::Identity();

  double translational_clip = 0.01;
  double rotational_clip = 0.05;
  translational_clip_min_ << -translational_clip, -translational_clip,
      -translational_clip;
  translational_clip_max_ << translational_clip, translational_clip,
      translational_clip;

  rotational_clip_min_ << -rotational_clip, -rotational_clip, -rotational_clip;
  rotational_clip_max_ << rotational_clip, rotational_clip, rotational_clip;

  double translational_Ki = 30;
  double rotational_Ki = 1;
  Ki_.setIdentity();
  Ki_.topLeftCorner(3, 3) << translational_Ki * Eigen::Matrix3d::Identity();
  Ki_.bottomRightCorner(3, 3) << rotational_Ki * Eigen::Matrix3d::Identity();

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

  error_i_.setZero();
  // goal_state_info->joint_positions << control_msg_.goal().q1(),
  // control_msg_.goal().q2(), control_msg_.goal().q3(),
  // control_msg_.goal().q4(), control_msg_.goal().q5(),
  // control_msg_.goal().q6(), control_msg_.goal().q7();
}

void HybridForcePositionController::get_max_error_(
    Eigen::Matrix<double, 6, 1> &DELTA) {
  Eigen::Matrix<double, 6, 1> max_cartesian_force_eigen;
  for (int i = 0; i < 6; ++i) {
    max_cartesian_force_eigen(i, 0) = max_cartesian_force_[i];
  }
  double denominator = kp_ + 2 * ki_ * 1000;
  DELTA = max_cartesian_force_eigen;
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

  // tau_J_d
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(
      robot_state.tau_J_d.data());

  // transform
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  // Current joint positions
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  // Current joint velocity
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  if (!initialized_) {
    force_ext_initial_ = force_ext;
    // set nullspace equilibrium configuration to initial q
    q_d_nullspace_ = q;

    initialized_ = true;
  }

  // FORCE CONTROL
  /*
  Eigen::Matrix<double, 6, 1> desired_force_torque, force_control;
  Eigen::Matrix<double, 7, 1> tau_force, tau_task, tau_cartesian_impedance,
      tau_cmd;

  desired_force_torque.setZero();
  desired_force_torque(2) = -1 * std::abs(desired_wrench_in_sensor_frame(2));
  // desired_force_torque.head<3>() = ;

  Eigen::Matrix<double, 6, 1> force_error, max_force_error;
  get_max_error_(max_force_error);
  force_error = desired_force_torque - force_ext + force_ext_initial_;
  force_error_ = force_error_ + 0.001 * force_error;
  force_control = desired_force_torque + ki_ * force_error_ + kp_ * force_error;
  // force_control = force_control.cwiseMin(max_force_error); // clamp error
  force_control << force_control(0), force_control(1), force_control(2), 0, 0,
      0;
  tau_force = jacobian.transpose() * force_control;
  */

  // Cartesian PD control
  Eigen::Matrix<double, 7, 1> tau_cartesian_impedance;

  // position error
  error_.head(3) << position - desired_pos_EE_in_base_frame;

  // clip pos
  for (int i = 0; i < 3; i++) {
    error_(i) = std::min(std::max(error_(i), translational_clip_min_(i)),
                         translational_clip_max_(i));
  }

  // orientation error
  if (desired_quat_EE_in_base_frame.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }

  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation *
                                      desired_quat_EE_in_base_frame.inverse());
  error_.tail(3) << error_quaternion.x(), error_quaternion.y(),
      error_quaternion.z();

  // clip rot
  for (int i = 0; i < 3; i++) {
    error_(i + 3) = std::min(std::max(error_(i + 3), rotational_clip_min_(i)),
                             rotational_clip_max_(i));
  }

  error_i_.head(3)
      << (error_i_.head(3) + error_.head(3)).cwiseMax(-0.1).cwiseMin(0.1);
  error_i_.tail(3)
      << (error_i_.tail(3) + error_.tail(3)).cwiseMax(-0.3).cwiseMin(0.3);

  tau_cartesian_impedance << jacobian.transpose() *
                                 (-cartesian_stiffness_ * error_ -
                                  cartesian_damping_ * (jacobian * dq) -
                                  Ki_ * error_i_);

  // Nullspace control
  //
  Eigen::Matrix<double, 7, 1> tau_nullspace;
  Eigen::Matrix<double, 6, 7> jacobian_transpose_pinv;
  // pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  Eigen::Matrix<double, 7, 1> dqe;
  Eigen::Matrix<double, 7, 1> qe;

  qe << q_d_nullspace_ - q;
  qe.head(1) << qe.head(1) * joint1_nullspace_stiffness_;
  dqe << dq;
  dqe.head(1) << dqe.head(1) * 2.0 * sqrt(joint1_nullspace_stiffness_);
  tau_nullspace << (Eigen::Matrix<double, 7, 7>::Identity() -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * qe -
                        (2.0 * sqrt(nullspace_stiffness_)) * dqe);
  // Desired torque
  tau_d << tau_cartesian_impedance + coriolis;
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  std::array<double, 7> tau_d_array{};
  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

  return tau_d_array;
}

void HybridForcePositionController::pseudoInverse(
    const Eigen::Matrix<double, 7, 6> &M_, Eigen::Matrix<double, 6, 7> &M_pinv_,
    bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::Matrix<double, 7, 6>> svd(
      M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::Matrix<double, 7, 6>>::SingularValuesType sing_vals_ =
      svd.singularValues();
  Eigen::Matrix<double, 7, 6> S_ =
      M_; // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
    S_(i, i) =
        (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::Matrix<double, 6, 7>(svd.matrixV() * S_.transpose() *
                                        svd.matrixU().transpose());
}

Eigen::Matrix<double, 7, 1> HybridForcePositionController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>
        &tau_J_d) { // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] +
        std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}
} // namespace controller
