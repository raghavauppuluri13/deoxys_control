// Copyright 2023 Raghava Uppuluri

#include "controllers/base_controller.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_HYBRID_FORCE_POSITION_CONTROLLER_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_HYBRID_FORCE_POSITION_CONTROLLER_H_

namespace controller {
class HybridForcePositionController : public BaseController {
protected:
  RpalHybridForcePositionControllerMessage control_msg_;

  // Force control PI
  double kp_{0.5};
  double desired_force_{7.0};
  double delta_tau_max_{1.0};
  double ki_{1.0};
  double kd_{1.0};
  double nullspace_stiffness_{0.2};
  double joint1_nullspace_stiffness_{100.0};
  std::array<double, 6> max_cartesian_force_{0.1, 0.1, 0.1, 0, 0, 0};
  static constexpr double kDeltaTauMax{1.0};

  // Cartesian impedance
  bool initialized_ = false;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 3, 1> translational_clip_min_;
  Eigen::Matrix<double, 3, 1> translational_clip_max_;
  Eigen::Matrix<double, 3, 1> rotational_clip_min_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Matrix<double, 3, 1> rotational_clip_max_;
  Eigen::Matrix<double, 6, 1> force_error_;
  Eigen::Matrix<double, 6, 1> error_;
  Eigen::Matrix<double, 6, 1> error_i_;
  Eigen::Matrix<double, 6, 1> force_ext_initial_;
  Eigen::Matrix<double, 6, 6> Ki_;
  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();
  void get_max_error_(Eigen::Matrix<double, 6, 1> &DELTA);

public:
  HybridForcePositionController();
  ~HybridForcePositionController();

  HybridForcePositionController(franka::Model &model);

  bool ParseMessage(const FrankaControlMessage &msg);

  // void ComputeGoal(const Eigen::Vector3d&, const Eigen::Quaterniond&,
  // Eigen::Vector3d&, Eigen::Quaterniond&);
  void ComputeGoal(const std::shared_ptr<StateInfo> &state_info,
                   std::shared_ptr<StateInfo> &goal_info);

  Eigen::Matrix<double, 7, 1>
  saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
                     const Eigen::Matrix<double, 7, 1>
                         &tau_J_d); // NOLINT (readability-identifier-naming)

  void pseudoInverse(const Eigen::Matrix<double, 7, 6> &M_,
                     Eigen::Matrix<double, 6, 7> &M_pinv_, bool damped);

  std::array<double, 7>
  Step(const franka::RobotState &robot_state,
       const Eigen::Vector3d &wrench_in_sensor_frame,
       const Eigen::Vector3d &desired_wrench_in_sensor_frame,
       const Eigen::Vector3d &desired_pos_EE_in_base_frame,
       const Eigen::Quaterniond &desired_quat_EE_in_base_frame);
};
} // namespace controller

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_HYBRID_FORCE_POSITION_CONTROLLER_H_
