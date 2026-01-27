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
/*
 * robot_kinematics.hpp
 *
 *  Created on: 2012/02/08
 *      Author: takeshita
 */
#ifndef TMC_ROBOT_KINEMATICS_MODEL_ROBOT_KINEMATICS_MODEL_HPP_
#define TMC_ROBOT_KINEMATICS_MODEL_ROBOT_KINEMATICS_MODEL_HPP_
#include <stdint.h>

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tmc_manipulation_types/manipulation_types.hpp>

namespace tmc_robot_kinematics_model {

/// Exception for invalid JointState
class JointStateError : public std::domain_error {
 public:
  explicit JointStateError(const std::string &error) :
    std::domain_error("error: " + error + " failed") {}
};

/// Kinematic model of the robot
/// Only the necessary functions for interference check for now
class IRobotKinematicsModel {
 public:
  using Ptr = std::shared_ptr<IRobotKinematicsModel>;
  virtual ~IRobotKinematicsModel() {}

  /// @brief  Initialize the robot model
  /// @param  [in] robot_description Robot model
  virtual void Initialize(const std::string& robot_description) = 0;

  /// @brief  Input the robot's position and orientation
  /// @param  [in] transform Robot's orientation
  virtual void SetRobotTransform(const Eigen::Affine3d& transform) = 0;

  /// @brief  Get the robot's position and orientation
  /// @return Eigen::Affine3d Robot's orientation
  virtual Eigen::Affine3d GetRobotTransform(void) const = 0;

  /// @brief  Specify the robot's joint name and input the joint angle
  /// @param  [in] angle Robot joint information
  /// @exception domain_error Throws an exception if a non-existent joint angle name is input
  /// @exception domain_error Throws an exception if the size of joint angles and joint names differ
  virtual void SetNamedAngle(
      const tmc_manipulation_types::JointState& angle) = 0;

  /// @brief  Get the robot's joint information
  /// @return JointState Robot's joint information
  virtual tmc_manipulation_types::JointState GetNamedAngle(void) const = 0;

  /// @brief  Specify the joint name to get the robot's joint information
  /// @param  [in] joint_names Vector of joint names
  /// @return JointState Robot's joint information
  /// @exception domain_error Throws an exception if a non-existent joint angle name is input
  virtual tmc_manipulation_types::JointState GetNamedAngle(
      const tmc_manipulation_types::NameSeq& joint_names) const = 0;

  /// @brief  Get the object's position and orientation
  /// @param  [in] name Name of the object to be retrieved
  /// @return Eigen::Affine3d Object's position and orientation
  /// @exception domain_error Throws an exception if a non-existent object name is input.
  virtual Eigen::Affine3d GetObjectTransform(
      const std::string& name) const = 0;
  /// Get the object's relative position and orientation
  virtual Eigen::Affine3d GetObjectRelativeTransform(
      const std::string& base_name, const std::string& name) const = 0;

  /// @brief  Dynamically add a frame
  /// @param  [in] parent_frame_name Name of the parent frame
  /// @param  [in] transform Position and orientation of the frame to be added relative to the parent frame
  /// @param  [in] new_frame_name Name of the frame to be added to retrieve the object name
  /// @exception domain_error Throws an exception if a non-existent object name is used as the parent frame.
  virtual void CreateFrame(
      const std::string& parent_frame_name, const Eigen::Affine3d& transform,
      const std::string& new_frame_name) = 0;

  /// @brief  Dynamically delete a frame
  /// @param  [in] frame_name Name of the frame to be deleted
  /// @exception domain_error Throws an exception if a frame name that has not been created is input.
  virtual void DestroyFrame(const std::string& frame_name) = 0;

  /// @brief  Get the Jacobian
  /// @param  [in] frame_name Target frame
  /// @param  [in] frame_to_end Offset from the target frame
  /// @param  [in] use_joints Target joint list
  /// @return Eigen::MatrixXd Jacobian
  /// @exception domain_error Throws an exception if a non-existent frame name is input.
  /// @exception domain_error Throws an exception if a non-existent joint name is input.
  virtual Eigen::MatrixXd GetJacobian(const std::string& frame_name,
                                      const Eigen::Affine3d& frame_to_end,
                                      const tmc_manipulation_types::NameSeq& use_joints) = 0;

  /// @brief  Get the Min and Max of the joints
  /// @param  [in] use_joints Names of the joints to be retrieved
  /// @param  [out] min Lower limit column of joint angles
  /// @param  [out] max Upper limit column of joint angles
  /// @exception domain_error Throws an exception if a non-existent joint name is input.
  virtual void GetMinMax(const tmc_manipulation_types::NameSeq& use_joints,
                         Eigen::VectorXd& min, Eigen::VectorXd& max) const = 0;
};
}  // namespace tmc_robot_kinematics_model
#endif /* ROBOT_KINEMATICS_MODEL_HPP_ */
