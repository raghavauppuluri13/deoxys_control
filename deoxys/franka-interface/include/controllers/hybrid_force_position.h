// Copyright 2023 Raghava Uppuluri

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "controllers/base_controller.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_HYBRID_FORCE_POSITION_CONTROLLER_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_HYBRID_FORCE_POSITION_CONTROLLER_H_

namespace controller {
class HybridForcePositionController : public BaseController {
protected:
  // Force control PI
  double desired_force_{0.0};
  double target_force_{0.0};
  double k_p_{0.0};
  double k_i_{0.0};
  Eigen::Matrix<double, 6, 1> force_ext_initial_;
  Eigen::Matrix<double, 6, 1> force_error_;
  static constexpr double kDeltaTauMax{1.0};

  // Wiggle motions
  double time_{0.0};
  double wiggle_frequency_x_{0.5};
  double wiggle_frequency_y_{0.5};
  double amplitude_wiggle_x_{0.1};
  double amplitude_wiggle_y_{0.1};

  // Cartesian impedance
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;

public:
  HybridForcePositionController();

  ~HybridForcePositionController();
};
} // namespace controller

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_HYBRID_FORCE_POSITION_CONTROLLER_H_
