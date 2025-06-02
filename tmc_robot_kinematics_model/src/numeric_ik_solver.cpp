/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
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
/// @brief Numerical solution IK solver
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
// Small bias used in Sugihara method from Sugihara 2009
const double kWn = 1e-3;

std::function<bool()> ReturnFalse = []() -> bool{ return false; };

/// Keep joint angles within limits
/// @param [IN] min Minimum joint angle
/// @param [IN] max Maximum joint angle
/// @param [IN,OUT] angle Joint angle
void SaturateAngle(const Eigen::VectorXd& min, const Eigen::VectorXd& max,
                   const std::vector<bool> is_continuous_joint,
                   Eigen::VectorXd& angle) {
  for (int32_t i = 0; i < angle.size(); ++i) {
    if (is_continuous_joint[i]) {
      // Since it's unlikely to loop around twice in numerical IK, this should be fine
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

/// Solve IK numerically fails if use_joints is 6 or less
/// @param [IN] request: IK input
/// @param [OUT] solution_angle_out: Solution posture
/// @param [OUT] Solution posture
IKResult NumericIKSolver::Solve(const IKRequest& request,
                                JointState& solution_angle_out,
                                Eigen::Affine3d& origin_to_end_out) {
  return Solve(request,
               ReturnFalse,
               solution_angle_out,
               origin_to_end_out);
}

/// Solve IK numerically fails if use_joints is 6 or less
/// @param [IN] request: IK input
/// @param [IN] interrupt Interrupt function
/// @param [OUT] solution_angle_out: Solution posture
/// @param [OUT] Solution posture
IKResult NumericIKSolver::Solve(const IKRequest& request,
                                std::function<bool()>& interrupt,
                                JointState& solution_angle_out,
                                Eigen::Affine3d& origin_to_end_out) {
  Eigen::Affine3d origin_to_base_dummy;
  return Solve(request,
               interrupt,
               solution_angle_out,
               origin_to_base_dummy,
               origin_to_end_out);
}

/// Solve IK numerically allowing base movement
/// @param [IN] request IK input
/// @param [OUT] solution_angle_out Solution posture
/// @param [OUT] origin_to_base_out Solution robot position and posture
/// @param [OUT] origin_to_end_out Solution posture
IKResult NumericIKSolver::Solve(const IKRequest& request,
                                JointState& solution_angle_out,
                                Eigen::Affine3d& origin_to_base_out,
                                Eigen::Affine3d& origin_to_end_out) {
  return Solve(request,
               ReturnFalse,
               solution_angle_out,
               origin_to_base_out,
               origin_to_end_out);
}

/// Solve IK numerically allowing base movement
/// @param [IN] request IK input
/// @param [IN] interrupt Interrupt function
/// @param [OUT] solution_angle_out Solution posture
/// @param [OUT] origin_to_base_out Solution robot position and posture
/// @param [OUT] origin_to_end_out Solution posture
IKResult NumericIKSolver::Solve(const IKRequest& request,
                                std::function<bool()>& interrupt,
                                JointState& solution_angle_out,
                                Eigen::Affine3d& origin_to_base_out,
                                Eigen::Affine3d& origin_to_end_out) {
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

  // Create position jacobian
  for (uint32_t i = 0; i < linear_base_movements.size(); ++i) {
    linear_base_jacobian.col(i) <<
        linear_base_movements[i], Eigen::Vector3d::Zero();
  }


  Eigen::MatrixXd jacobian_with_base(kSE3Dim, total_dof);

  // Throw exception if number of joints + base degrees of freedom is 6 or less
  if (total_dof < kSE3Dim) {
    throw std::invalid_argument(
        "use_joints's + base dof has to be at least 6.");
  }

  Eigen::MatrixXd wn = Eigen::MatrixXd::Identity(total_dof, total_dof);
  // If weight size matches DOF, weight wn
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

  ref_origin_to_frame = request.ref_origin_to_end * (request.frame_to_end).inverse();

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

    origin_to_current = robot_model_->GetObjectTransform(request.frame_name);
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
      origin_to_end_out = origin_to_current * request.frame_to_end;
      origin_to_base_out = origin_to_base;
      return kSuccess;
    } else if (fabs(delta - delta_old) < converge_threshold_) {
      solution_angle_out = current_joint;
      origin_to_end_out = origin_to_current * request.frame_to_end;
      origin_to_base_out = origin_to_base;
      return kConverge;
    }
    if (!request.use_joints.empty()) {
      jacobian = robot_model_->GetJacobian(request.frame_name,
                                           request.frame_to_end,
                                           request.use_joints);
    }

    // Create rotation jacobian
    for (uint32_t i = 0; i < rotational_base_movements.size(); ++i) {
      rotational_base_jacobian.col(i) <<
          rotational_base_movements[i].cross(origin_to_current.translation() -
                                             origin_to_base.translation()),
          rotational_base_movements[i];
    }

    // Add base movement jacobian
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
    // Calculate base position correction
    Eigen::Vector3d mod_base_pos = Eigen::Vector3d::Zero();
    for (uint32_t i = 0; i < linear_base_movements.size(); ++i) {
      mod_base_pos += angle_diff(dof + i) * linear_base_movements[i];
    }
    // Calculate base rotation correction
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
  origin_to_end_out = origin_to_current * request.frame_to_end;
  return kMaxItr;
}

/// Solve IK numerically with a maximum of one solution
/// @param [IN] request: IK input
/// @param [OUT] responses_out: IK solution
IKResult NumericIKSolver::Solve(const IKRequest& request,
                                std::vector<IKResponse>& responses_out) {
  return Solve(request, ReturnFalse, responses_out);
}

/// Solve IK numerically with a maximum of one solution
/// @param [IN] request: IK input
/// @param [IN] interrupt Interrupt function
/// @param [OUT] responses_out: IK solution
IKResult NumericIKSolver::Solve(const IKRequest& request,
                                std::function<bool()>& interrupt,
                                std::vector<IKResponse>& responses_out) {
  IKResponse response;
  if (Solve(request, interrupt,
            response.solution_angle, response.origin_to_base, response.origin_to_end) == kSuccess) {
    responses_out.clear();
    responses_out.push_back(response);
    return kSuccess;
  } else {
    return kFail;
  }
}

}  // namespace tmc_robot_kinematics_model


#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(tmc_robot_kinematics_model::NumericIKSolver,
                       tmc_robot_kinematics_model::IKSolver)


/// @brief Wrapper function for generating IKRequest for ctypes
/// @param [in] movement Base type
/// @return req_p Pointer to IKRequest
void* create_request(tmc_manipulation_types::BaseMovementType movement) {
  tmc_robot_kinematics_model::IKRequest* req =
    new tmc_robot_kinematics_model::IKRequest(movement);
  void* req_p = reinterpret_cast<void*>(req);
  return req_p;
}

/// @brief Wrapper function for setting the name of the frame to solve IK for ctypes
/// @param [in,out] req_p Pointer to IKRequest
/// @param [in] frame_name Frame name
void set_req_frame_name(void* req_p, char* frame_name) {
  struct tmc_robot_kinematics_model::IKRequest* req =
    (struct tmc_robot_kinematics_model::IKRequest*)req_p;
  req->frame_name = frame_name;
}

/// @brief Wrapper function for setting the transformation from the frame to end coordinates for ctypes
/// @param [in,out] req_p Pointer to IKRequest
/// @param [in] mat Transformation from frame to end coordinates. 4x4 matrix
void set_req_frame_to_end(void* req_p, double* mat) {
  Eigen::Matrix4d  matrix;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      matrix(i, j) = mat[i * 4 + j];
    }
  }
  struct tmc_robot_kinematics_model::IKRequest* req =
    (struct tmc_robot_kinematics_model::IKRequest*)req_p;
  req->frame_to_end.matrix() = matrix;
}

/// @brief Wrapper function for setting the transformation from origin to base in IKRequest for ctypes
/// @param [in,out] req_p Pointer to IKRequest
/// @param [in] mat Transformation from origin to base. 4x4 matrix
void set_req_origin_to_base(void* req_p, double* mat) {
  Eigen::Matrix4d  matrix;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      matrix(i, j) = mat[i * 4 + j];
    }
  }
  struct tmc_robot_kinematics_model::IKRequest* req =
    (struct tmc_robot_kinematics_model::IKRequest*)req_p;
  req->origin_to_base.matrix() = matrix;
}

/// @brief Wrapper function for setting the joint names at the start of IK in IKRequest for ctypes
/// @param [in,out] req_p Pointer to IKRequest
/// @param [in] angle_names Joint names
/// @param [in] num_elements Number of joints to set
void set_req_initial_angle_name(void* req_p, char* angle_names[], int num_elements) {
  tmc_manipulation_types::NameSeq use_name;
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  req->initial_angle.name.resize(num_elements);
  for (int index = 0; index < num_elements; ++index) {
    char* angle_name = angle_names[index];
    use_name.push_back(angle_name);
  }
  req->initial_angle.name = use_name;
  req->use_joints = use_name;
}

/// @brief Wrapper function for setting the joint angle sequence at the start of IK in IKRequest for ctypes
/// @param [in,out] req_p Pointer to IKRequest
/// @param [in] pos Joint angle sequence
/// @param [in] len_pos Number of joints to set
void set_req_initial_angle_position(void* req_p, float pos[], int len_pos) {
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  req->initial_angle.position.resize(len_pos);

  for (int i = 0; i < len_pos; i++) {
    req->initial_angle.position[i] = pos[i];
  }
}

/// @brief Wrapper function for setting weights in IKRequest for ctypes
/// @param [in,out] req_p Pointer to IKRequest
/// @param [in] weight Weight array
/// @param [in] len_weight Length of the weight array
void set_req_weight(void* req_p, float weight[], int len_weight) {
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  req->weight.resize(len_weight);
  for (int i = 0; i < len_weight; i++) {
    req->weight[i] = weight[i];
  }
}

/// @brief Wrapper function for setting the transformation from origin to end coordinates in IKRequest for ctypes
/// @param [in,out] req_p Pointer to IKRequest
/// @param [in] mat Transformation from origin to end coordinates. 4x4 matrix
void set_req_ref_origin_to_end(void* req_p, double* mat) {
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  Eigen::Matrix4d  matrix;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      matrix(i, j) = mat[i * 4 + j];
    }
  }
  req->ref_origin_to_end.matrix() = matrix;
}

/// @brief Wrapper function to return a pointer to a JointState object for ctypes
/// @return Pointer to the JointState object
void* jointstate() {
  JointState* js = new JointState;

  void* res = reinterpret_cast<void*>(js);
  return res;
}

/// @brief Wrapper function to return a pointer to an Affine3d object for ctypes
/// @return Pointer to the Affine3d object
void* affine3d() {
  Eigen::Affine3d* affine3d = new Eigen::Affine3d;
  return reinterpret_cast<void*>(affine3d);
}

/// @brief Wrapper function to create an IK solver object for ctypes
/// @param [in] robot_p Pointer to the robot model for IK (tarp3_wrapper)
/// @param [in] max_itr Maximum number of iterations
/// @param [in] epsilon Allowable error
/// @param [in] converge_threshold Considered converged if change is less than this in one iteration
/// @return Pointer to the IK solver object
void* create_solver(void* robot_p, int max_itr,
                    float epsilon, float converge_threshold) {
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr* robot =
    reinterpret_cast<
    tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr*>(robot_p);
  tmc_robot_kinematics_model::IKSolver::Ptr* numeric_solver
    = new tmc_robot_kinematics_model::IKSolver::Ptr;
  numeric_solver->reset(
    new tmc_robot_kinematics_model::NumericIKSolver(
        tmc_robot_kinematics_model::IKSolver::Ptr(),
        *robot,
        max_itr,
        epsilon,
        converge_threshold));
  return reinterpret_cast<void*>(numeric_solver);
}

/// @brief Wrapper function for solving IK for ctypes
/// @param [in] solver_p Pointer to the solver
/// @param [out] solution_p Pointer to the result joint angle sequence of solved IK
/// @param [out] origin_to_base_solution_p Pointer to the result transformation from origin to base of solved IK
/// @param [out] origin_to_hand_result_p Pointer to the result transformation from origin to hand of solved IK
/// @param [out] req_p Pointer to the request
void solve(void* solver_p, void* solution_p,
           void* origin_to_base_solution_p,
           void* origin_to_hand_result_p,
           void* req_p) {
  tmc_robot_kinematics_model::IKSolver::Ptr* numeric_solver =
    reinterpret_cast<tmc_robot_kinematics_model::IKSolver::Ptr*>(solver_p);
  JointState* solution = reinterpret_cast<JointState*>(solution_p);
  Eigen::Affine3d* origin_to_hand_result =
    reinterpret_cast<Eigen::Affine3d*>(origin_to_hand_result_p);
  Eigen::Affine3d* origin_to_base_solution =
    reinterpret_cast<Eigen::Affine3d*>(origin_to_base_solution_p);
  struct tmc_robot_kinematics_model::IKRequest* req =
    reinterpret_cast<struct tmc_robot_kinematics_model::IKRequest*>(req_p);
  tmc_robot_kinematics_model::IKResult result;
  // Solve.
  result = (*numeric_solver)->Solve(*req,
                                    *solution,
                                    *origin_to_base_solution,
                                    *origin_to_hand_result);
  // Print result.
  if (result == tmc_robot_kinematics_model::kSuccess) {
  } else {
    std::cout << "ik cannot be solved" << std::endl;
  }
}

/// @brief Wrapper function to return the joint angle sequence result of solved IK for ctypes
/// @param [in] sol_p Pointer to the joint angle sequence (JointState)
/// @param [out] result Joint angle sequence
/// @param [in] len Number of joints used
void get_joint_angle(void* sol_p,
                     float* result,
                     int len) {
  JointState* solution = reinterpret_cast<JointState*>(sol_p);
  for (int i=0; i < len; i++) {
    result[i] = solution->position[i];
  }
}

/// @brief Wrapper function to return the transformation from origin to base result of solved IK for ctypes
/// @param [in] origin_to_base Pointer to the transformation from origin to base (Affine3d)
/// @param [out] result Transformation from origin to base (4x4 matrix (serialized))
void get_origin_to_base(void* origin_to_base,
                        double* result) {
  Eigen::Affine3d* otb = reinterpret_cast<Eigen::Affine3d*>(origin_to_base);
  Eigen::MatrixXd  mat = otb->matrix();
  for (int i=0; i < 4; i++) {
    for (int j=0; j < 4; j++) {
      result[4*i + j] = mat(i, j);
    }
  }
}
