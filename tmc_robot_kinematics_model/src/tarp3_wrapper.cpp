/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <tarp3/tarp_matrix.h>
#include <tarp3/tarp_phase.h>
#include <tarp3/tarp_robot.h>
#include <tarp3_urdf/tarp_urdf.h>

#include <tmc_robot_kinematics_model/tarp3_wrapper.hpp>

using tmc_manipulation_types::JointState;
using tmc_manipulation_types::NameSeq;
using tmc_robot_kinematics_model::IRobotKinematicsModel;

namespace {
inline Eigen::Affine3d TarpTrans2Eigen(const tarp_matrix3_t& tarp_rot,
                                const tarp_vector3_t& tarp_pos) {
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.linear() = Eigen::Map<const Eigen::Matrix<
    double, 3, 3, Eigen::RowMajor> > (tarp_rot[0]);
  transform.translation() = Eigen::Map<const Eigen::Vector3d>(tarp_pos);
  return transform;
}

inline void EigenTrans2Tarp(const Eigen::Affine3d& transform,
                     tarp_matrix3_t tarp_rot,
                     tarp_vector3_t tarp_vec) {
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >
      rot_map(tarp_rot[0]);
  Eigen::Map<Eigen::Vector3d> pos_map(tarp_vec);

  rot_map = transform.linear();
  pos_map = transform.translation();
}

inline Eigen::MatrixXd TarpMatrix2Eigen(
    const tarp_matrix_t* tarp_mat) {
  Eigen::Map<
    const Eigen::Matrix<double, Eigen::Dynamic,
                        Eigen::Dynamic, Eigen::RowMajor> >
      tarp_map(tarp_mat->data, tarp_mat->row, tarp_mat->col);
  return tarp_map;
}
}  // anonymous namespace

namespace tmc_robot_kinematics_model {

class Tarp3Wrapper::Impl {
 public:
  explicit Impl(const std::string& filename) :
      robot_(tarp_robot_load_urdf_string(filename.c_str())),
      origin_to_robot_(Eigen::Affine3d::Identity()) {
    if (robot_ == NULL) {
      throw Tarp3Error(filename + " cannot load as tarp model.");
    }
    num_joints_ = tarp_robot_get_joint_numb(robot_);
  }
  virtual ~Impl() {
    delete robot_;
  }
  void SetRobotTransform(const Eigen::Affine3d& origin_to_robot) {
    origin_to_robot_ = origin_to_robot;
  }

  Eigen::Affine3d GetRobotTransform(void) const {
    return origin_to_robot_;
  }

  void SetNamedAngle(const JointState& angle) {
    if (angle.name.size() != angle.position.size()) {
      throw Tarp3Error("Joint name size and position size are mismatch.");
    }
    for (uint32_t i = 0; i < angle.name.size(); ++i) {
      tarp_joint_t* joint = static_cast<tarp_joint_t*>(
          tarp_robot_lookup(robot_, angle.name[i].c_str()));
      if (joint == NULL) {
        throw Tarp3Error("Joint "+ angle.name[i] + " is not found.");
      }
      tarp_joint_set_dis(joint, angle.position(i));
    }
    tarp_robot_update_dis(robot_, 0.0);
  }

  JointState GetNamedAngle(const NameSeq& joint_names) const {
    JointState angle;
    angle.name = joint_names;
    angle.position.resize(joint_names.size());
    for (uint32_t i = 0; i < joint_names.size(); ++i) {
      tarp_joint_t* joint = static_cast<tarp_joint_t*>(
          tarp_robot_lookup(robot_, joint_names[i].c_str()));
      if (joint == NULL) {
        throw Tarp3Error("Joint "+ angle.name[i] + " is not found.");
      }
      angle.position(i) = tarp_joint_get_dis(joint);
    }
    return angle;
  }

  JointState GetNamedAngle() const {
    JointState angle;
    tarp_joint_t* joints[num_joints_];
    int32_t return_joint_number = 0;
    return_joint_number =
        tarp_robot_get_joint_array(robot_, joints, num_joints_);
    std::vector<double> position;
    uint32_t j = 0;
    for (uint32_t i = 0; i < return_joint_number; ++i) {
      switch (tarp_joint_get_type(joints[i])) {
        case TARP_JOINT_TYPE_ROT:
        case TARP_JOINT_TYPE_POS:
          {
            angle.name.push_back(tarp_joint_get_name(joints[i]));
            position.push_back(tarp_joint_get_dis(joints[i]));
            ++j;
            break;
          }
        default:
          break;
      }
      angle.position = Eigen::Map<Eigen::VectorXd>(
          &position[0], position.size());
    }
    return angle;
  }

  Eigen::Affine3d GetObjectTransform(
      const std::string& object_name) const {
    Eigen::Affine3d robot_to_object = Eigen::Affine3d::Identity();
    tarp_gizmo_t* object = static_cast<tarp_gizmo_t*>(
          tarp_robot_lookup(robot_, object_name.c_str()));
    if (object == NULL) {
      throw Tarp3Error("object "+ object_name + " is not found.");
    }
    tarp_vector3_t pos;
    tarp_matrix3_t rot;
    tarp_gizmo_get_act_pos_dis(object, pos, 'r');
    tarp_gizmo_get_act_rot_dis(object, rot, 'r');
    robot_to_object = TarpTrans2Eigen(rot, pos);
    return origin_to_robot_ * robot_to_object;
  }

  Eigen::Affine3d GetObjectRelativeTransform(
      const std::string& base_name,
      const std::string& object_name) const {
    return GetObjectTransform(base_name).inverse() *
        GetObjectTransform(object_name);
  }

  void CreateFrame(
      const std::string& parent_frame_name, const Eigen::Affine3d& transform,
      const std::string& new_frame_name) {
    tarp_frame_t* frame = tarp_frame_create();
    tarp_gizmo_t* parent_object =
        static_cast<tarp_gizmo_t*>(
            tarp_robot_lookup(robot_, parent_frame_name.c_str()));
    if (parent_object == NULL) {
      throw Tarp3Error("object "+ parent_frame_name + " is not found.");
    }
    tarp_frame_set_name(frame, new_frame_name.c_str());
    tarp_gizmo_attach(parent_object, reinterpret_cast<tarp_gizmo_t*>(frame));
    tarp_vector3_t pos;
    tarp_matrix3_t rot;
    EigenTrans2Tarp(transform, rot, pos);
    tarp_frame_set_pos_dis(frame, pos, 'p');
    tarp_gizmo_set_ref_rot_dis(reinterpret_cast<tarp_gizmo_t*>(frame),
                               rot, 'p');
    tarp_robot_append(robot_, reinterpret_cast<tarp_gizmo_t*>(frame));
    tarp_robot_update_dis(robot_, 0.0);
  }

  void DestroyFrame(const std::string& frame_name) {
    tarp_frame_t* frame =
        static_cast<tarp_frame_t*>(
            tarp_robot_lookup(robot_, frame_name.c_str()));
    if (frame == NULL) {
      throw Tarp3Error("frame "+ frame_name + " is not found.");
    }
    tarp_robot_remove_by_name(robot_, frame_name.c_str());
    tarp_frame_delete(frame);
  }

  Eigen::MatrixXd GetJacobian(const std::string& frame_name,
                              const Eigen::Affine3d& frame_to_end,
                              const NameSeq& use_joints) {
    std::string end_frame = frame_name + "_jacobian";
    CreateFrame(frame_name, frame_to_end, end_frame);
    tarp_matrix_t* jacob_pos = tarp_matrix_create(3, use_joints.size());
    tarp_matrix_t* jacob_rot = tarp_matrix_create(3, use_joints.size());

    // Define phases for robot operation
    tarp_phase_t* phase = tarp_phase_create();
    tarp_phase_set_robot(phase, robot_);

    tarp_frame_t* frame =
        static_cast<tarp_frame_t*>(tarp_robot_lookup(robot_, end_frame.c_str()));

    // Constraints for position (x, y, z) (3 degrees of freedom)
    tarp_rivet_frame_pos_t* rivet_pos = tarp_rivet_frame_pos_create();
    // Set frame in rivet_frame_pos_dir
    tarp_rivet_frame_pos_set_frame(rivet_pos, frame);

    // Constraints for orientation (3 degrees of freedom)
    tarp_rivet_frame_rot_t* rivet_rot = tarp_rivet_frame_rot_create();
    // Set frame in rivet_frame_rot
    tarp_rivet_frame_rot_set_frame(rivet_rot, frame);

    // Add rivet to phase
    tarp_phase_push_rivet(phase, reinterpret_cast<tarp_rivet_t*>(rivet_pos));
    tarp_phase_push_rivet(phase, reinterpret_cast<tarp_rivet_t*>(rivet_rot));

    std::vector<tarp_strap_joint_t*> strap_list;
    for (uint32_t i = 0; i < use_joints.size(); ++i) {
      tarp_strap_joint_t* strap;
      tarp_joint_t* joint;
      strap = tarp_strap_joint_create();
      strap_list.push_back(strap);
      joint = static_cast<tarp_joint_t*>(
          tarp_robot_lookup(robot_, use_joints[i].c_str()));
      if (joint == NULL) {
        throw Tarp3Error("Joint "+ use_joints[i] + " is not found.");
      }
      tarp_strap_joint_set_joint(strap, joint);
      // Add strap to phase
      tarp_phase_push_strap(phase, reinterpret_cast<tarp_strap_t*>(strap));
    }
    // Update Jacobian matrix of phase
    tarp_phase_update_jacob(phase);
    tarp_rivet_get_jacob(reinterpret_cast<tarp_rivet_t*>(rivet_pos), jacob_pos);
    tarp_rivet_get_jacob(reinterpret_cast<tarp_rivet_t*>(rivet_rot), jacob_rot);

    // Convert Jacobian from tarp3, which is based on base, to origin basis
    Eigen::MatrixXd jacobian_pos = origin_to_robot_.linear() *
        TarpMatrix2Eigen(jacob_pos);
    Eigen::MatrixXd jacobian_rot = origin_to_robot_.linear() *
        TarpMatrix2Eigen(jacob_rot);

    Eigen::MatrixXd jacobian(6, use_joints.size());
    jacobian << jacobian_pos, jacobian_rot;

    DestroyFrame(end_frame);
    tarp_matrix_delete(jacob_pos);
    tarp_matrix_delete(jacob_rot);
    tarp_phase_delete(phase);

    tarp_rivet_frame_pos_delete(rivet_pos);
    tarp_rivet_frame_rot_delete(rivet_rot);

    for (auto* strap, strap_list) {
      tarp_strap_joint_delete(strap);
    }
    return jacobian;
  }

  void GetMinMax(const NameSeq& use_joints,
                 Eigen::VectorXd& min,
                 Eigen::VectorXd& max) const {
    min.resize(use_joints.size());
    max.resize(use_joints.size());
    for (uint32_t i = 0; i < use_joints.size(); ++i) {
      tarp_joint_t* joint = static_cast<tarp_joint_t*>(
          tarp_robot_lookup(robot_, use_joints[i].c_str()));
      if (joint == NULL) {
        throw Tarp3Error("Joint "+ use_joints[i] + " is not found.");
      }
      min(i) = joint->min;
      max(i) = joint->max;
    }
  }

  // For eigen fixed-length members
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  tarp_robot_t* robot_;
  Eigen::Affine3d origin_to_robot_;
  uint32_t num_joints_;
  std::map<std::string, tarp_frame_t*> additional_frames_;
};

/// @brief Constructor. Create robot model from configuration file
/// @param [in] robot_model_config Path to robot model configuration file
Tarp3Wrapper::Tarp3Wrapper(const std::string& robot_model_config) :
    pimpl_(new Impl(robot_model_config)) {
}

/// @brief Destructor
Tarp3Wrapper::~Tarp3Wrapper() {
}

/// @brief Get robot's position and orientation
Eigen::Affine3d Tarp3Wrapper::GetRobotTransform(void) const {
  return pimpl_->GetRobotTransform();
}


/// @brief Input robot's position and orientation
void Tarp3Wrapper::SetRobotTransform(const Eigen::Affine3d& transform) {
  pimpl_->SetRobotTransform(transform);
}

/// @brief Specify robot's joint name and input its joint angle
void Tarp3Wrapper::SetNamedAngle(const JointState& angle) {
  pimpl_->SetNamedAngle(angle);
}

/// @brief Get robot's joint name and its angle
///        If called without arguments, returns all joints.
JointState Tarp3Wrapper::GetNamedAngle(void) const {
  return pimpl_->GetNamedAngle();
}

/// @brief  Get robot's joint name and its angle
JointState Tarp3Wrapper::GetNamedAngle(const NameSeq& joint_names) const {
  return pimpl_->GetNamedAngle(joint_names);
}

/// @brief  Get object's position and orientation
Eigen::Affine3d Tarp3Wrapper::GetObjectTransform(
    const std::string& name) const {
  return pimpl_->GetObjectTransform(name);
}

/// @brief  Get object's relative position and orientation
/// @param  [in] base_name Reference object name
/// @param  [in] name Object name to retrieve
/// @return Eigen::Affine3d Object's position and orientation
Eigen::Affine3d Tarp3Wrapper::GetObjectRelativeTransform(
    const std::string& base_name,
    const std::string& name) const {
  return pimpl_->GetObjectRelativeTransform(base_name, name);
}

/// @brief  Dynamically add frame
void Tarp3Wrapper::CreateFrame(
    const std::string& parent_frame_name, const Eigen::Affine3d& transform,
    const std::string& new_frame_name) {
  return pimpl_->CreateFrame(parent_frame_name, transform,
                             new_frame_name);
}

/// @brief  Dynamically remove frame
void Tarp3Wrapper::DestroyFrame(const std::string& frame_name) {
  pimpl_->DestroyFrame(frame_name);
}

/// @brief Get Jacobian
/// @param [in] frame_name Specified frame name
/// @param [in] frame_to_end Offset from specified frame
/// @param [in] use_joints Joint angles to use
/// @retval Jacobian from robot_base to end
Eigen::MatrixXd Tarp3Wrapper::GetJacobian(const std::string& frame_name,
                                          const Eigen::Affine3d& frame_to_end,
                                          const NameSeq& use_joints) {
  return pimpl_->GetJacobian(frame_name, frame_to_end, use_joints);
}

/// @brief Get minimum and maximum of joints.
/// @param [in] use_joints Joint angles to use
/// @param [out] min Lower limit
/// @param [out] max Upper limit
void Tarp3Wrapper::GetMinMax(const NameSeq& use_joints,
                             Eigen::VectorXd& min,
                             Eigen::VectorXd& max) const {
  return pimpl_->GetMinMax(use_joints, min, max);
}
}  // namespace tmc_robot_kinematics_model

/// @brief Get minimum and maximum of joints.
/// @param [in] model_path Path to robot's model file (urdf)
/// @return robot Pointer to robot model
void* load_xml_create_robot(char* model_path) {
  std::string xml_string;
  std::fstream xml_file(model_path, std::fstream::in);
  while (xml_file.good()) {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }

  IRobotKinematicsModel::Ptr* robot = new IRobotKinematicsModel::Ptr;
  robot->reset(new tmc_robot_kinematics_model::Tarp3Wrapper(xml_string));
  return reinterpret_cast<void*>(robot);
}
