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
/// @file     numeric_ik_sover.hpp
/// @brief    Numerical solution IK solver
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2012.2.28
/// @note     [1.0.0] 2012.2.28 Newly created

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <tmc_robot_kinematics_model/numeric_ik_solver.hpp>
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

using tmc_manipulation_types::JointState;

namespace {
constexpr uint32_t kDefaultMaxItr = 1000;
constexpr double kDefaultEpsilon = 1.0e-3;
constexpr double kDefaultConvergeThreshold = 1.0e-10;

const uint32_t kSE3Dim = 6;
// Micro bias used in Sugihara method from Sugihara 2009
const double kWn = 1e-3;

std::function<bool()> ReturnFalse = []() -> bool{ return false; };

/// Keep joint angles within limits
/// @param [IN] min Minimum joint angle
/// @param [IN] min Maximum joint angle
/// @param [IN,OUT] angle Joint angle
void SaturateAngle(const Eigen::VectorXd& min, const Eigen::VectorXd& max,
                   const std::vector<bool> is_continuous_joint,
                   Eigen::VectorXd& angle) {
  for (int32_t i = 0; i < angle.size(); ++i) {
    if (is_continuous_joint[i]) {
      // Numerical IK is unlikely to exceed two full rotations, so this should be fine
      if (angle(i) > M_PI) {
        angle(i) -= 2.0 * M_PI;
      } else if (angle(i) < -M_PI) {
        angle(i) += 2.0 * M_PI;
      }
    } else {
      angle(i) = std::max(min(i), angle(i));
      angle(i) = std::min(max(i), angle(i));
    }
  }
}
}  // anonymous namespace

namespace tmc_robot_kinematics_model {

NumericIKSolver::NumericIKSolver()
    : max_itr_(kDefaultMaxItr), epsilon_(kDefaultEpsilon), converge_threshold_(kDefaultConvergeThreshold) {}

void NumericIKSolver::set_robot_description(const std::string& robot_description) {
  robot_model_ = std::make_shared<PinocchioWrapper>(robot_description);
}

/// Solve IK numerically, with at most one solution
/// @param [IN] request: IK input
/// @param [OUT] responses_out: IK solution
IKResult NumericIKSolver::Solve(const IKRequest& request,
                                std::vector<IKResponse>& responses_out) {
  return Solve(request, ReturnFalse, responses_out);
}

/// Solve IK numerically, with at most one solution
/// @param [IN] request: IK input
/// @param [IN] interrupt Interrupt function
/// @param [OUT] responses_out: IK solution
IKResult NumericIKSolver::Solve(const IKRequest& request,
                                std::function<bool()>& interrupt,
                                std::vector<IKResponse>& responses_out) {
  IKResponse response;
  response.origin_to_ends.resize(1);
  const auto result = SolveImpl(
      request, interrupt, response.solution_angle, response.origin_to_base, response.origin_to_ends[0]);
  if (result == kSuccess) {
    responses_out.clear();
    responses_out.push_back(response);
    return kSuccess;
  } else {
    return result;
  }
}

/// Solve IK numerically, allowing base movement
/// @param [IN] request IK input
/// @param [IN] interrupt Interrupt function
/// @param [OUT] solution_angle_out Solution posture
/// @param [OUT] origin_to_base_out Robot position and posture of the solution
/// @param [OUT] origin_to_end_out Solution posture
IKResult NumericIKSolver::SolveImpl(const IKRequest& request,
                                    std::function<bool()>& interrupt,
                                    JointState& solution_angle_out,
                                    Eigen::Affine3d& origin_to_base_out,
                                    Eigen::Affine3d& origin_to_end_out) {
  IKRequest::IKTargetFrame target_frame;
  if (request.target_frames.size() > 0) {
    target_frame = request.target_frames[0];
  } else {
    throw std::invalid_argument("target_frame is not specified.");
  }

  double delta = 0.0;
  double delta_old = 0.0;
  uint32_t dof = request.use_joints.size();
  robot_model_->SetRobotTransform(request.origin_to_base);
  robot_model_->SetNamedAngle(request.initial_angle);

  Eigen::Affine3d origin_to_current(Eigen::Affine3d::Identity());
  Eigen::Affine3d origin_to_base(request.origin_to_base);
  Eigen::Affine3d ref_origin_to_frame(Eigen::Affine3d::Identity());
  Eigen::Affine3d ref_to_current(Eigen::Affine3d::Identity());
  Eigen::MatrixXd jacobian(kSE3Dim, dof);
  Eigen::VectorXd angle_diff(dof);
  Eigen::Matrix<double, kSE3Dim, 1> diff;
  Eigen::AngleAxisd diff_rot;
  Eigen::Vector3d diff_pos;
  JointState current_joint;
  Eigen::VectorXd angle_max(dof);
  Eigen::VectorXd angle_min(dof);

  std::vector<Eigen::Vector3d> linear_base_movements =
      request.linear_base_movements;
  std::vector<Eigen::Vector3d> rotational_base_movements =
      request.rotational_base_movements;

  uint32_t total_dof = dof +
      linear_base_movements.size() + rotational_base_movements.size();
  Eigen::MatrixXd linear_base_jacobian(
      kSE3Dim, linear_base_movements.size());
  Eigen::MatrixXd rotational_base_jacobian(
      kSE3Dim, rotational_base_movements.size());

  // Create position Jacobian
  for (uint32_t i = 0; i < linear_base_movements.size(); ++i) {
    linear_base_jacobian.col(i) <<
        linear_base_movements[i], Eigen::Vector3d::Zero();
  }


  Eigen::MatrixXd jacobian_with_base(kSE3Dim, total_dof);

  // Throw exception if the number of joints + base degrees of freedom is 6 or less
  if (total_dof < kSE3Dim) {
    throw std::invalid_argument(
        "use_joints's + base dof has to be at least 6.");
  }

  Eigen::MatrixXd wn = Eigen::MatrixXd::Identity(total_dof, total_dof);
  // If the size of weight matches DOF, apply weight to wn
  if (request.weight.size() == static_cast<int32_t>(total_dof)) {
    for (uint32_t i = 0; i < dof; ++i) {
      // Throw exception if weight is negative
      if (request.weight(i) < 0.0) {
        throw std::invalid_argument(
            "ik joint weights have to be positive double.");
      } else {
        wn(i, i) = request.weight(i);
      }
    }
    for (uint32_t i = dof; i < dof + linear_base_movements.size(); ++i) {
      wn(i, i) = request.weight(i);
    }
    for (uint32_t i = dof + linear_base_movements.size();
         i < total_dof; ++i) {
      wn(i, i) = request.weight(i);
    }
  }

  ref_origin_to_frame = target_frame.ref_origin_to_end * target_frame.frame_to_end.inverse();

  robot_model_->GetMinMax(request.use_joints, angle_min, angle_max);

  std::vector<bool> is_continuous_joint(dof);
  for (auto i = 0; i < request.use_joints.size(); ++i) {
    if (std::find(request.continuous_joints.begin(), request.continuous_joints.end(), request.use_joints[i]) ==
            request.continuous_joints.end()) {
      is_continuous_joint[i] = false;
    } else {
      is_continuous_joint[i] = true;
    }
  }

  for (uint32_t i = 0; i < max_itr_; ++i) {
    if (interrupt()) {
      return kInterruption;
    }

    origin_to_current = robot_model_->GetObjectTransform(target_frame.frame_name);
    current_joint = robot_model_->GetNamedAngle(request.use_joints);
    Eigen::Quaterniond diff_quat(origin_to_current.linear().transpose()
                                 * ref_origin_to_frame.linear());
    diff_rot = Eigen::AngleAxisd(diff_quat.normalized());
    diff_pos = ref_origin_to_frame.translation()
        - origin_to_current.translation();
    diff << diff_pos,
        origin_to_current.linear() * diff_rot.angle() * diff_rot.axis();

    delta_old = delta;
    delta = diff.norm();
    if (delta < epsilon_) {
      solution_angle_out = current_joint;
      origin_to_end_out = origin_to_current * target_frame.frame_to_end;
      origin_to_base_out = origin_to_base;
      return kSuccess;
    } else if (fabs(delta - delta_old) < converge_threshold_) {
      solution_angle_out = current_joint;
      origin_to_end_out = origin_to_current * target_frame.frame_to_end;
      origin_to_base_out = origin_to_base;
      return kConverge;
    }
    if (!request.use_joints.empty()) {
      jacobian = robot_model_->GetJacobian(target_frame.frame_name,
                                           target_frame.frame_to_end,
                                           request.use_joints);
    }

    // Create rotation Jacobian
    for (uint32_t i = 0; i < rotational_base_movements.size(); ++i) {
      rotational_base_jacobian.col(i) <<
          rotational_base_movements[i].cross(origin_to_current.translation() -
                                             origin_to_base.translation()),
          rotational_base_movements[i];
    }

    // Add Jacobian for base movement
    if (request.use_joints.empty()) {
      if (linear_base_movements.empty()) {
        if (rotational_base_movements.empty()) {
        } else {
          jacobian_with_base <<
              rotational_base_jacobian;
        }
      } else {
        if (rotational_base_movements.empty()) {
          jacobian_with_base <<
              linear_base_jacobian;
        } else {
          jacobian_with_base <<
              linear_base_jacobian, rotational_base_jacobian;
        }
      }
    } else {
      if (linear_base_movements.empty()) {
        if (rotational_base_movements.empty()) {
          jacobian_with_base = jacobian;
        } else {
          jacobian_with_base <<
              jacobian, rotational_base_jacobian;
        }
      } else {
        if (rotational_base_movements.empty()) {
          jacobian_with_base <<
              jacobian, linear_base_jacobian;
        } else {
          jacobian_with_base <<
              jacobian, linear_base_jacobian, rotational_base_jacobian;
        }
      }
    }
    // Newton-Rapson
    // angle_diff = jacobian_with_base.transpose() * (jacobian_with_base *
    //                                      jacobian_with_base.transpose()).inverse() * diff;
    // LM method [Sugihara 2009]
    angle_diff = (jacobian_with_base.transpose() * jacobian_with_base
                  + (diff.transpose() * diff)(0, 0)
                  * wn
                  + kWn * wn).inverse()
        * jacobian_with_base.transpose() * diff;
    current_joint.position += angle_diff.head(dof);
    // Modified to respect joint angle limits
    SaturateAngle(angle_min, angle_max, is_continuous_joint, current_joint.position);
    // Calculate correction for base position
    Eigen::Vector3d mod_base_pos = Eigen::Vector3d::Zero();
    for (uint32_t i = 0; i < linear_base_movements.size(); ++i) {
      mod_base_pos += angle_diff(dof + i) * linear_base_movements[i];
    }
    // Calculate correction for base rotation
    for (uint32_t i = 0; i < rotational_base_movements.size(); ++i) {
      double angle = angle_diff(dof + linear_base_movements.size() + i);
      origin_to_base = origin_to_base *
          Eigen::AngleAxisd(
              angle,
              origin_to_base.rotation() * rotational_base_movements[i]);
    }
    origin_to_base = Eigen::Translation3d(mod_base_pos) * origin_to_base;

    robot_model_->SetRobotTransform(origin_to_base);
    robot_model_->SetNamedAngle(current_joint);
  }
  solution_angle_out = current_joint;
  origin_to_base_out = origin_to_base;
  origin_to_end_out = origin_to_current * target_frame.frame_to_end;
  return kMaxItr;
}

}  // namespace tmc_robot_kinematics_model

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(tmc_robot_kinematics_model::NumericIKSolver,
                       tmc_robot_kinematics_model::IKSolver)
