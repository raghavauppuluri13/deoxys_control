// Copyright 2023 Raghava Uppuluri

#include <Eigen/src/Core/Matrix.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "controllers/base_controller.h"
#include "utils/shared_state.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_HYBRID_FORCE_POSITION_CONTROLLER_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_HYBRID_FORCE_POSITION_CONTROLLER_H_

namespace controller {
class HybridForcePositionController : public BaseController {
protected:
  RpalHybridForcePositionControllerMessage control_msg_;

  // Force control PI
  double kp_{0.0};
  double ki_{0.0};
  static constexpr double kDeltaTauMax{1.0};

  // Cartesian impedance
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;

public:
  HybridForcePositionController();
  HybridForcePositionController(franka::Model &model);

  ~HybridForcePositionController();

  std::array<double, 7> HybridForcePositionController::Step(
      const franka::RobotState &robot_state,
      const Eigen::Vector3d &wrench_in_sensor_frame,
      const Eigen::Vector3d &desired_wrench_in_sensor_frame,
      const Eigen::Vector3d &desired_pos_EE_in_base_frame,
      const Eigen::Quaterniond &desired_quat_EE_in_base_frame);
}; // namespace controller

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_HYBRID_FORCE_POSITION_CONTROLLER_H_
