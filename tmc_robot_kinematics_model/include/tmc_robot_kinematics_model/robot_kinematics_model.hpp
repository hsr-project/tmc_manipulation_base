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
/// For now, only the necessary functions for the interference check are included
class IRobotKinematicsModel {
 public:
  using Ptr = std::shared_ptr<IRobotKinematicsModel>;
  virtual ~IRobotKinematicsModel() {}

  /// @brief  Initialization of the robot model
  /// @param  [in] robot_description Robot model
  virtual void Initialize(const std::string& robot_description) = 0;

  /// @brief  Input the robot's position and orientation
  /// @param  [in] transform Robot posture
  virtual void SetRobotTransform(const Eigen::Affine3d& transform) = 0;

  /// @brief  Retrieve the robot's position and orientation
  /// @return Eigen::Affine3d Robot posture
  virtual Eigen::Affine3d GetRobotTransform(void) const = 0;

  /// @brief  Specify the robot joint name and input the joint angle
  /// @param  [in] angle Robot joint information
  /// @exception domain_error Throws exception when nonexistent joint angle name is inputted
  /// @exception domain_error Throws exception if the size of joint angle and joint name differs
  virtual void SetNamedAngle(
      const tmc_manipulation_types::JointState& angle) = 0;

  /// @brief  Retrieve robot joint information
  /// @return JointState Robot joint information
  virtual tmc_manipulation_types::JointState GetNamedAngle(void) const = 0;

  /// @brief  Retrieve robot joint information by specifying joint names
  /// @param  [in] joint_names Vector of joint names
  /// @return JointState Robot joint information
  /// @exception domain_error Throws exception when nonexistent joint angle name is inputted
  virtual tmc_manipulation_types::JointState GetNamedAngle(
      const tmc_manipulation_types::NameSeq& joint_names) const = 0;

  /// @brief  Retrieve the position and orientation of an object
  /// @param  [in] name Name of the object to retrieve
  /// @return Eigen::Affine3d Position and orientation of the object
  /// @exception domain_error Throws exception when nonexistent object name is inputted.
  virtual Eigen::Affine3d GetObjectTransform(
      const std::string& name) const = 0;
  /// Retrieve the relative position and orientation of the object
  virtual Eigen::Affine3d GetObjectRelativeTransform(
      const std::string& base_name, const std::string& name) const = 0;

  /// @brief  Dynamically add a frame
  /// @param  [in] parent_frame_name Name of the parent frame
  /// @param  [in] transform Position and orientation of the frame to be added relative to the parent frame
  /// @param  [in] new_frame_name Name of the frame to be added
  /// @exception domain_error Throws exception if a nonexistent object name is set as the parent frame.
  virtual void CreateFrame(
      const std::string& parent_frame_name, const Eigen::Affine3d& transform,
      const std::string& new_frame_name) = 0;

  /// @brief  Dynamically delete a frame
  /// @param  [in] frame_name Name of the frame to delete
  /// @exception domain_error Throws exception if a frame name not created is inputted.
  virtual void DestroyFrame(const std::string& frame_name) = 0;

  /// @brief  Retrieve the Jacobian
  /// @param  [in] frame_name Target frame
  /// @param  [in] frame_to_end Offset from the target frame
  /// @param  [in] use_joints Target joint list
  /// @return Eigen::MatrixXd Jacobian
  /// @exception domain_error Throws exception when nonexistent frame name is inputted.
  /// @exception domain_error Throws exception when nonexistent joint name is inputted.
  virtual Eigen::MatrixXd GetJacobian(const std::string& frame_name,
                                      const Eigen::Affine3d& frame_to_end,
                                      const tmc_manipulation_types::NameSeq& use_joints) = 0;

  /// @brief  Retrieve Min and Max of joints
  /// @param  [in] use_joints Names of joints to retrieve
  /// @param  [out] min Lower limit column of joint angle
  /// @param  [out] max Upper limit column of joint angle
  /// @exception domain_error Throws exception when nonexistent joint name is inputted.
  virtual void GetMinMax(const tmc_manipulation_types::NameSeq& use_joints,
                         Eigen::VectorXd& min, Eigen::VectorXd& max) const = 0;
};
}  // namespace tmc_robot_kinematics_model
#endif /* ROBOT_KINEMATICS_MODEL_HPP_ */
