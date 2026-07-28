/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// @file     ik_sover.hpp
/// @brief    IK solver
#ifndef ROBOT_KINEMATICS_MODEL_IK_SOLVER_HPP__
#define ROBOT_KINEMATICS_MODEL_IK_SOLVER_HPP__

#include <memory>
#include <string>
#include <vector>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>

namespace tmc_robot_kinematics_model {
enum IKResult {
  kSuccess,
  kConverge,
  kMaxItr,
  kFail,
  kInterruption
};

/// IK request
struct IKRequest {
  IKRequest() {}
  /// Specify BaseMovementType when there is Base movement
  explicit IKRequest(tmc_manipulation_types::BaseMovementType _base_type) : base_type(_base_type) {
    switch (_base_type) {
      case tmc_manipulation_types::kFloat:
        linear_base_movements.push_back(Eigen::Vector3d::UnitX());
        linear_base_movements.push_back(Eigen::Vector3d::UnitY());
        linear_base_movements.push_back(Eigen::Vector3d::UnitZ());
        rotational_base_movements.push_back(Eigen::Vector3d::UnitX());
        rotational_base_movements.push_back(Eigen::Vector3d::UnitY());
        rotational_base_movements.push_back(Eigen::Vector3d::UnitZ());
        break;
      case tmc_manipulation_types::kPlanar:
        linear_base_movements.push_back(Eigen::Vector3d::UnitX());
        linear_base_movements.push_back(Eigen::Vector3d::UnitY());
        rotational_base_movements.push_back(Eigen::Vector3d::UnitZ());
        break;
      case tmc_manipulation_types::kRailX:
        linear_base_movements.push_back(Eigen::Vector3d::UnitX());
        break;
      case tmc_manipulation_types::kRailY:
        linear_base_movements.push_back(Eigen::Vector3d::UnitY());
        break;
      case tmc_manipulation_types::kRailZ:
        linear_base_movements.push_back(Eigen::Vector3d::UnitZ());
        break;
      case tmc_manipulation_types::kRotationX:
        rotational_base_movements.push_back(Eigen::Vector3d::UnitX());
        break;
      case tmc_manipulation_types::kRotationY:
        rotational_base_movements.push_back(Eigen::Vector3d::UnitY());
        break;
      case tmc_manipulation_types::kRotationZ:
        rotational_base_movements.push_back(Eigen::Vector3d::UnitZ());
        break;
      case tmc_manipulation_types::kNone:
        break;
      default:
        break;
    }
  }
  /// Robot position and orientation
  Eigen::Affine3d origin_to_base;
  /// Initial posture. It's better to include all joints.
  /// Otherwise, the already set joint angles will be used.
  tmc_manipulation_types::JointState initial_angle;
  /// Target joint names
  std::vector<std::string> use_joints;
  /// Weights for each joint. Ignored if not the same length as use_joints+base_dof.
  Eigen::VectorXd weight;
  /// Translational base movement
  std::vector<Eigen::Vector3d> linear_base_movements;
  /// Rotational base movement
  std::vector<Eigen::Vector3d> rotational_base_movements;
  /// Continuous joint names
  std::vector<std::string> continuous_joints;

  // Target position and orientation information
  struct IKTargetFrame {
    std::string frame_name;
    Eigen::Affine3d frame_to_end;
    Eigen::Affine3d ref_origin_to_end;

    IKTargetFrame(const std::string& _frame_name,
                  const Eigen::Affine3d& origin_to_end)
        : frame_name(_frame_name),
          ref_origin_to_end(origin_to_end),
          frame_to_end(Eigen::Affine3d::Identity()) {}
    IKTargetFrame() : frame_to_end(Eigen::Affine3d::Identity()) {}
  };
  std::vector<IKTargetFrame> target_frames;
  tmc_manipulation_types::BaseMovementType base_type;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct IKResponse {
  tmc_manipulation_types::JointState solution_angle;
  Eigen::Affine3d origin_to_base;
  std::vector<Eigen::Affine3d> origin_to_ends;
};

/// IK interface
/// Customizable IK methods using chain of responsibility
/// Example: Use analytical solutions for requests involving a 6-axis arm
class IKSolver {
 public:
  using Ptr = std::shared_ptr<IKSolver>;
  IKSolver() {}
  /// @param [IN] successor IK to pass to the next in Next
  explicit IKSolver(IKSolver::Ptr successor) : successor_(successor) {}
  virtual ~IKSolver() {}

  void set_successor(IKSolver::Ptr successor) { successor_ = successor;}
  virtual void set_robot_description(const std::string& robot_description) {}

  /// Solve IK
  /// @param [IN] request IK request
  /// @param [OUT] responses_out IK solutions
  /// @retval kSuccess At least one IK solution was obtained
  /// @retval kFail Failure, no solution
  virtual IKResult Solve(const IKRequest& request,
                         std::vector<IKResponse>& responses_out) {
    std::function<bool()> func = []() -> bool{ return false; };
    return Solve(request, func, responses_out);
  }

  /// Solve IK
  /// @param [IN] request IK request
  /// @param [IN] interrupt Interrupt function
  /// @param [OUT] responses_out IK solutions
  /// @retval kSuccess At least one IK solution was obtained
  /// @retval kFail Failure, no solution
  virtual IKResult Solve(const IKRequest& request,
                         std::function<bool()>& interrupt,
                         std::vector<IKResponse>& responses_out) {
    return Next_(request, interrupt, responses_out);
  }

 protected:
  /// Delegate to successor
  /// @param [IN] request IK request
  /// @param [IN] interrupt Interrupt function
  /// @param [OUT] responses_out IK solutions
  /// @retval kSuccess At least one IK solution was obtained
  /// @retval kFail Failure, no solution
  IKResult Next_(const IKRequest& request,
                 std::function<bool()>& interrupt,
                 std::vector<IKResponse>& responses_out) {
    if (successor_) {
      return successor_->Solve(request, interrupt, responses_out);
    } else {
      return kFail;
    }
  }

 private:
  Ptr successor_;
};
}  // namespace tmc_robot_kinematics_model
#endif
