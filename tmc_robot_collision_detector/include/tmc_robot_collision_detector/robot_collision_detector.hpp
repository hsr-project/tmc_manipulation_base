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
/// @file robot_collision_detector.hpp
/// @brief Perform interference check using robot model
#ifndef TMC_ROBOT_COLLISION_DETECTOR_ROBOT_COLLISION_DETECTOR_HPP_
#define TMC_ROBOT_COLLISION_DETECTOR_ROBOT_COLLISION_DETECTOR_HPP_

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

// pinocchio must be included before boost
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

#include <Eigen/Core>  // NOLINT

#include <tmc_collision_detector/collision_detector.hpp>
#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_robot_collision_detector/robot_collision_detector_common.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>
#include "tmc_robot_collision_detector/collision_detector_config.hpp"
#include "tmc_robot_collision_detector/object_for_dump.hpp"

#if defined(__GNUC__)
#define DEPRECATED __attribute__((deprecated))
#elif defined(_WIN32)
#define DEPRECATED __declspec(deprecated)
#else
#define DEPRECATED
#endif

namespace tmc_robot_collision_detector {

/// CollisionDetector for robots
class RobotCollisionDetector {
 public:
  using Ptr = std::shared_ptr<RobotCollisionDetector>;

  /// @brief Constructor. Performs various initializations
  /// @param [in] robot_model Kinematic model of the robot
  /// @param [in] robot_model_config Path to the robot model configuration file
  /// @param [in] robot_collision_config Path to the interference check configuration file
  /// @param [in] engine Name of the engine used for interference check
  /// @par Behavior:
  /// - Create a robot kinematic model.
  /// - Load interference check settings.
  /// - Create interference check space.
  /// - If any of these fail, an exception is thrown.
  RobotCollisionDetector(const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot_model,
                         const std::string& robot_model_config,
                         const std::string& robot_collision_config,
                         const std::string& engine);

  /// @brief Constructor. Performs various initializations
  /// @param [in] robot_model_config Path to the robot model configuration file
  /// @param [in] robot_collision_config Path to the interference check configuration file
  /// @param [in] engine Name of the engine used for interference check
  /// @par Behavior:
  /// - Create a robot kinematic model.
  /// - Load interference check settings.
  /// - Create interference check space.
  /// - If any of these fail, an exception is thrown.
  RobotCollisionDetector(const std::string& robot_model_config,
                         const std::string& robot_collision_config,
                         const std::string& engine);

  /// @brief Constructor. Performs various initializations
  /// @param [in] robot_model_config Path to the robot model configuration file
  /// @param [in] robot_collision_config Path to the interference check configuration file
  /// @param [in] engine Name of the engine used for interference check
  /// @param [in] model_file_type Type of the robot model
  /// @par Behavior:
  /// - Create a robot kinematic model.
  /// - Load interference check settings.
  /// - Create interference check space.
  /// - If any of these fail, an exception is thrown.
  RobotCollisionDetector(const std::string& robot_model_config,
                         const std::string& robot_collision_config,
                         const std::string& engine,
                         ModelFileType model_file_type);

  virtual ~RobotCollisionDetector() {}

  /// @brief Input joint angle information
  /// @param [in] named_angle Joint angle information
  /// @par Behavior:
  /// - Apply named_angle to the robot kinematic model.
  /// - Update the interference check space based on the updated robot model.
  void SetRobotNamedAngle(
      const tmc_manipulation_types::JointState& named_angle);

  /// @brief Retrieve joint angle information
  /// @return tmc_manipulation_types::JointState Joint angle information
  /// @par Behavior:
  /// - Retrieve joint angle information for all joints from the robot kinematic model.
  tmc_manipulation_types::JointState GetRobotNamedAngle(void) const  {
    return robot_model_->GetNamedAngle();
  }

  /// @brief Retrieve joint angle information
  /// @param [in] name Name of the joint to retrieve information for
  /// @return tmc_manipulation_types::JointState Joint angle information
  /// @par Behavior:
  /// - Retrieve joint angle information for the specified joint name from the robot kinematic model.
  tmc_manipulation_types::JointState GetRobotNamedAngle(
      const tmc_manipulation_types::NameSeq& name) const {
    return robot_model_->GetNamedAngle(name);
  }

  /// @brief Retrieve joint Min and Max
  /// @param [in] use_joints Joint names to retrieve
  /// @param [out] min Lower limit of joint angles
  /// @param [out] max Upper limit of joint angles
  void GetRobotAngleMinMax(
      const tmc_manipulation_types::NameSeq& use_joints,
      Eigen::VectorXd& min,
      Eigen::VectorXd& max) const {
    robot_model_->GetMinMax(use_joints, min, max);
  }

  /// @brief Input robot pose
  /// @param [in] Eigen::Affine3d Robot pose
  /// @par Behavior:
  /// - Retrieve robot pose from the robot kinematic model.
  void SetRobotTransform(const Eigen::Affine3d& origin_to_robot);

  /// @brief Retrieve robot pose
  /// @return Eigen::Affine3d Robot pose
  /// @par Behavior:
  /// - Retrieve robot pose from the robot kinematic model.
  Eigen::Affine3d GetRobotTransform(void) const {
    return robot_model_->GetRobotTransform();
  }

  /// @brief Create external objects
  /// @param [in] parameters Information of external objects const
  /// @par Behavior:
  /// - Verify that the number of child object postures and shape information is the same.
  ///   If different, throw an exception.
  /// - Verify if the object name is set; if not, throw an exception.
  /// - Verify whether an object with the same name already exists.
  ///   If already exists, throw an exception.
  /// - Generate an object in the interference check space,
  ///   and add to the external object list.
  /// @attention
  /// - Created objects are subject to interference checks.
  /// - Child object names will be parent object name + # + serial number.
  void CreateOuterObject(
      const tmc_manipulation_types::OuterObjectParameters& parameters);

  /// @brief Construct an environment consisting of Cuboids
  /// @param [in] map Vector of Cuboids
  /// @param [in] enable_flag Enable/disable flag for Cuboid interference check upon creation
  ///             true=enable, false=disable
  /// @par Behavior:
  /// - Create Cuboids in the interference check space according to the map.
  ///   If the name of Cuboid is empty, throw an exception.
  /// - Disable interference check for created Cuboids.
  /// - Add to the external object list.
  /// - Add to the Cuboid list.
  /// @attention
  /// - If you only want to do a simple interference check, set enable_flag to false
  /// - Group of Cuboids will be CUBOID
  void CreateCuboids(const tmc_manipulation_types::CuboidSeq& map,
                     bool enable_flag);

  /// @brief Construct an environment consisting of Cuboids
  /// @param [in] map Vector of Cuboids
  /// @param [in] enable_flag Enable/disable flag for Cuboid interference check upon creation
  ///             true=enable, false=disable
  /// @param [in] cuboid_group_name Group name for Cuboids
  /// @par Behavior:
  /// - Create Cuboids in the interference check space according to the map.
  ///   If the name of Cuboid is empty, throw an exception.
  /// - Disable interference check for created Cuboids.
  /// - Add to the external object list.
  /// - Add to the Cuboid list.
  /// @attention
  /// - If you only want to do a simple interference check, set enable_flag to false
  void CreateCuboids(const tmc_manipulation_types::CuboidSeq& map,
                     bool enable_flag,
                     const std::string& cuboid_group_name);

  /// @brief Dispose of external objects
  /// @param [in] object_name Name of the object to dispose of
  /// @par Behavior:
  /// - Check if object_name is a robot part.
  ///   If it is a robot part, do nothing.
  /// - Check if object_name is a grasping object.
  ///   If it is a grasping object, do nothing.
  /// - Check if object_name exists in the external object list.
  ///   If it does not exist, throw an exception.
  /// - Dispose of the object from the interference check space.
  /// @attention
  /// - Only child objects cannot be disposed of.
  /// - Cuboids can be disposed of.
  void DestroyOuterObject(const std::string& object_name);

  /// @brief Dispose of all external objects
  /// @par Behavior:
  /// - Dispose of all external objects from the interference check space.
  /// @attention
  /// - All grasped objects will also be disposed of.
  /// - All Cuboids will also be disposed of.
  void DestroyAllOuterObject(void);

  /// @brief Dispose of environment consisting of Cuboids
  /// @par Behavior:
  /// - Dispose of Cuboids existing in the Cuboid list from the interference check space.
  /// - Dispose of the Cuboid list.
  void DestroyCuboids();

  /// @brief Retrieve object information
  /// @param [in] object_name Name of the object
  /// @return OuterObjectParameters Object information
  /// @par Behavior:
  /// - For external objects, retrieve and return information from the external object list.
  /// - For others, retrieve and return information from the interference check space.
  /// - If object_name does not exist, throw an exception.
  tmc_manipulation_types::OuterObjectParameters GetObjectParameter(
      const std::string& object_name) const;

  /// @brief Retrieve information of all external objects
  /// @return OuterObjectParametersSeq Information of all external objects
  /// @par Behavior:
  /// - Return the external object list.
  tmc_manipulation_types::OuterObjectParametersSeq
  GetAllOuterObjectParameters(void) const;

  /// @brief Enable object interference check
  /// @param [in] object_name Object name
  /// @par Behavior:
  /// - Enable interference check of object in interference check space.
  /// @attention
  /// - If parent object is specified, all child objects will be targeted.
  /// - If child object is specified, only the child object will be targeted.
  /// - If Cuboid is specified, it will be removed from the Cuboid list
  ///   and become a regular external object.
  void EnableCollisionObject(const std::string& object_name);

  /// @brief Disable object interference check
  /// @param [in] object_name Object name
  /// @par Behavior:
  /// - Disable interference check of object in interference check space.
  /// @attention
  /// - If parent object is specified, all child objects will be targeted.
  /// - If child object is specified, only the child object will be targeted.
  /// - If Cuboid is specified, it will be removed from the Cuboid list
  ///   and become a regular external object.
  void DisableCollisionObject(const std::string& object_name);

  /// @brief Change object's group
  /// @param [in] object_name Object name
  /// @param [in] category Group bit to be changed
  /// @par Behavior:
  /// - Change the group of the object in the interference check space.
  /// @attention
  /// - If parent object is specified, all child objects will be targeted.
  /// - If child object is specified, only the child object will be targeted.
  /// - If Cuboid is specified, it will be removed from the Cuboid list
  ///   and become a regular external object.
  void SetObjectGroup(const std::string& object_name, uint16_t category);

  /// @brief Reset object's group to default
  /// @param [in] object_name Object name
  /// @par Behavior:
  /// - Retrieve default group bit from interference check settings
  /// - Change the group of the object in the interference check space.
  /// @attention
  /// - If parent object is specified, all child objects will be targeted.
  /// - If child object is specified, only the child object will be targeted.
  /// - If Cuboid is specified, it will be removed from the Cuboid list
  ///   and become a regular external object.
  void SetObjectDefaultGroup(const std::string& object_name);

  /// @brief Retrieve object's group
  /// @param [in] object_name Object name
  /// @return uint16_t Group bit
  /// @par Behavior:
  /// - Retrieve current group bit from interference check space
  /// @attention
  /// - If parent object is specified, the first child object will be targeted.
  uint16_t GetObjectGroup(const std::string& object_name) const;

  /// @brief Retrieve object's default group
  /// @param [in] object_name Object name
  /// @return uint16_t Group bit
  /// @par Behavior:
  /// - Retrieve default group bit from interference check settings
  uint16_t GetObjectDefaultGroup(const std::string& object_name) const;

  /// @brief Change object's filter
  /// @param [in] object_name Object name
  /// @param [in] filter Filter bit to be changed
  /// @par Behavior:
  /// - Change the filter of the object in the interference check space.
  /// @attention
  /// - If parent object is specified, all child objects will be targeted.
  /// - If child object is specified, only the child object will be targeted.
  /// - If Cuboid is specified, it will be removed from the Cuboid list
  ///   and become a regular external object.
  void SetObjectFilter(const std::string& object_name, uint16_t filter);

  /// @brief Reset object's filter to default
  /// @param [in] object_name Object name
  /// @par Behavior:
  /// - Retrieve default filter bit from interference check settings
  /// - Change the filter of the object in the interference check space.
  /// @attention
  /// - If parent object is specified, all child objects will be targeted.
  /// - If child object is specified, only the child object will be targeted.
  /// - If Cuboid is specified, it will be removed from the Cuboid list
  ///   and become a regular external object.
  void SetObjectDefaultFilter(const std::string& object_name);

  /// @brief Retrieve object's filter
  /// @param [in] object_name Object name
  /// @return uint16_t Filter bit
  /// @par Behavior:
  /// - Retrieve current filter bit from interference check space
  /// @attention
  /// - If parent object is specified, the first child object will be targeted.
  uint16_t GetObjectFilter(const std::string& object_name) const;

  /// @brief Retrieve object's default filter
  /// @param [in] object_name Object name
  /// @return uint16_t Filter bit
  /// @par Behavior:
  /// - Retrieve default filter bit from interference check settings
  uint16_t GetObjectDefaultFilter(const std::string& object_name) const;

  /// @brief Add a pair of objects to be excluded from interference checks
  /// @param [in] object_name1 Object name
  /// @param [in] object_name2 Object name
  /// @par Behavior:
  /// - Add/remove from the exclusion pair list held by the interference check class
  ///   and additional pair list.
  /// @attention
  /// - To cancel Enable, specify the same pair with Disable.
  /// - If parent object is specified, all child objects will be targeted.
  /// - If child object is specified, only the child object will be targeted.
  /// - If Cuboid is specified, it will be removed from the Cuboid list
  ///   and become a regular external object.
  void DisableCollisionCheckObjectToObject(const std::string& object_name1,
                                           const std::string& object_name2);

  /// @brief Add a pair of objects for interference checks
  /// @param [in] object_name1 Object name
  /// @param [in] object_name2 Object name
  /// @par Behavior:
  /// - Add/remove from the exclusion pair list held by the interference check class
  ///   and additional pair list.
  /// @attention
  /// - To cancel Enable, specify the same pair with Disable.
  /// - If parent object is specified, all child objects will be targeted.
  /// - If child object is specified, only the child object will be targeted.
  /// - If Cuboid is specified, it will be removed from the Cuboid list
  ///   and become a regular external object.
  void EnableCollisionCheckObjectToObject(const std::string& object_name1,
                                          const std::string& object_name2);

  /// @brief Add object-group pair to be excluded from interference checks
  /// @param [in] object_name Object name
  /// @param [in] group_name Group name
  /// @par Behavior:
  /// - Add/remove from the exclusion pair list held by the interference check class
  ///   and additional pair list.
  /// @attention
  /// - To cancel Enable, specify the same pair with Disable.
  /// - If parent object is specified, all child objects will be targeted.
  /// - If child object is specified, only the child object will be targeted.
  /// - If Cuboid is specified, it will be removed from the Cuboid list
  ///   and become a regular external object.
  void DisableCollisionCheckObjectToGroup(const std::string& object_name,
                                          const std::string& group_name);

  /// @brief Add object-group pair for interference checks
  /// @param [in] object_name Object name
  /// @param [in] group_name Group name
  /// @par Behavior:
  /// - Add/remove from the exclusion pair list held by the interference check class
  ///   and additional pair list.
  /// @attention
  /// - To cancel Enable, specify the same pair with Disable.
  /// - If parent object is specified, all child objects will be targeted.
  /// - If child object is specified, only the child object will be targeted.
  /// - If Cuboid is specified, it will be removed from the Cuboid list
  ///   and become a regular external object.
  void EnableCollisionCheckObjectToGroup(const std::string& object_name,
                                         const std::string& group_name);

  /// @brief Change the filter of objects belonging to each group to exclude from interference checks
  ///
  /// @param [in] group_name1 Group name
  /// @param [in] group_name2 Group name
  /// @par Behavior:
  /// - Retrieve list of objects belonging to each group.
  /// - Operate object filter to avoid interference checks.
  void DisableCollisionCheckGroupToGroup(const std::string& group_name1,
                                         const std::string& group_name2);

  /// @brief Change the filter of objects belonging to each group to include in interference checks
  ///
  /// @param [in] group_name1 Group name
  /// @param [in] group_name2 Group name
  /// @par Behavior:
  /// - Retrieve list of objects belonging to each group.
  /// - Operate object filter to include in interference checks.
  void EnableCollisionCheckGroupToGroup(const std::string& group_name1,
                                        const std::string& group_name2);

  /// @brief Set object position and orientation
  /// @param [in] object_name Name of the object to set
  /// @param [in] origin_to_object Position and orientation to set
  /// @par Behavior:
  /// - Check if object_name is a robot part.
  ///   If it is a robot part, do nothing.
  /// - Check if object_name is a grasping object.
  ///   If it is a grasping object, do nothing.
  /// - Check if object_name exists in the external object list.
  ///   If it does not exist, throw an exception.
  /// - Change the position and orientation of the object in the interference check space.
  /// - Update posture of the object information.
  /// @attention
  /// - Only child objects cannot have their posture changed separately.
  ///   They will follow the change of the parent object.
  /// - If Cuboid is specified, it will be removed from the Cuboid list
  ///   and become a regular external object.
  void SetObjectTransform(const std::string& object_name,
                          const Eigen::Affine3d& origin_to_object);

  /// @brief Retrieve object position
  /// @param [in] object_name Object name
  /// @return Eigen::Affine3d Object posture
  /// @par Behavior:
  /// - Check if object_name exists in the external object list.
  ///   If it exists, return the posture from the object information.
  /// - Retrieve object position and orientation from interference check space.
  ///   If retrieved, return its posture.
  /// - Retrieve object position and orientation from the robot model.
  ///   If retrieved, return its posture.
  /// - If unable to retrieve from any source, throw an exception.
  /// @attention
  /// - Postures of child objects and parts only existing in the robot model (joints, etc.),
  ///   Cuboid postures can also be acquired.
  Eigen::Affine3d GetObjectTransform(const std::string& object_name) const;

  /// @brief Grasp an object
  /// @param [in] object_name Name of the object to be grasped
  /// @param [in] frame_name Name of the object to grasp
  /// @param [in] held_frame_to_object
  ///             Relative position of the grasped object based on the object to grasp
  /// @param [in] held_group_name Group name of the object to grasp
  /// @par Behavior:
  /// - Check if already grasping object_name.
  /// - Check if object_name is a robot part.
  ///   If it is a robot part, do nothing.
  /// - Check if object_name exists in the external object list.
  ///   If it does not exist, throw an exception.
  /// - Change the group/filter of the grasped object according to the group name of the object to grasp.
  /// - Add frame_name, held_frame_to_object,
  ///   held_group_name to the object information.
  /// - Update position of the grasped object in interference check space.
  /// - Update the list of grasped objects.
  /// - Only child objects cannot be grasped separately.
  /// @attention
  /// - If Cuboid is specified, it will be removed from the Cuboid list
  ///   and become a grasping object.
  /// @brief Grasp an object, the group of the grasping object will be the same as frame_name
  void HoldObject(const std::string& object_name,
                  const std::string& frame_name,
                  const Eigen::Affine3d& held_frame_to_object,
                  const std::string& held_group_name);

  /// @param [in] object_name Name of the object to be grasped
  /// @param [in] frame_name Name of the object to grasp
  ///             Relative position of the grasped object based on the object to grasp
  /// @param [in] held_frame_to_object
  /// @par Behavior:
  /// - Retrieve group name from frame_name.
  /// - Obtain the group name from frame_name.
  void HoldObject(const std::string& object_name,
                  const std::string& frame_name,
                  const Eigen::Affine3d& held_frame_to_object);

  /// @brief Release the grasped object
  /// @param [in] object_name Name of the object to release
  /// @par Behavior:
  /// - Update the list of grasped objects.
  ///   If object_name does not exist in the list of grasped objects, throw an exception.
  /// - Reset the category/filter of the grasped object to default.
  void ReleaseObject(const std::string& object_name);

  /// @brief Release all grasped objects
  /// @par Behavior:
  /// - Release all grasped objects.
  void ReleaseAllObject(void);

  /// @brief Interference check, stop checking upon detection of interference
  /// @return bool true if interference detected
  virtual bool CheckCollision();

  /// @brief Interference check, output names of interfering objects
  /// @param [in] end_flag false until the end
  ///             Perform interference check and create interference pair list
  /// @param [out] dst_contact_pair Interference pair
  /// @return bool true if interference detected
  bool CheckCollision(bool end_flag,
                      std::vector<PairString>& dst_contact_pair);

  /// @brief Search nearby objects for each robot component
  /// @param [in] extend_length Upper limit of distance to nearby objects
  /// @param [in] top_n Number of objects for which accurate distance is determined by GJK algorithm
  /// @param [out] dst_result_list Information of nearby objects for each robot part
  /// @return bool true if interference detected
  /// @par Behavior:
  /// - For robot parts and grasped objects,
  ///   obtain nearby objects inside/outside respectively.
  /// - Detailed search described in the README of collision detector.
  bool CheckClosestObject(double extend_length, int32_t top_n,
                          std::vector<ClosestObject>& dst_result_list);

  /// @brief Search for nearby objects of object_name
  /// @param [in] object_name Name of the object to search
  /// @param [in] extend_length Upper limit of distance to nearby objects
  /// @param [in] top_n Number of objects for which accurate distance is determined by GJK algorithm
  /// @param [out] dst_result Information of nearby objects for each robot part
  /// @return bool true if interference detected
  /// @par Behavior:
  /// - Detailed search described in the README of collision detector.
  bool CheckClosestObject(const std::string& object_name, double extend_length,
                          int32_t top_n, ClosestObject& dst_result);

  /// @brief Check if two objects are interfering
  /// @param [in] nameA Object A
  /// @param [in] nameB Object B
  /// @param [in] dst_point Contact point of A and B
  /// @param [in] dst_normal Normal of the contact point of A and B
  /// @return bool true if interfering
  bool CheckCollisionPair(
      const std::string& nameA, const std::string& nameB,
      Eigen::Vector3d& dst_point, Eigen::Vector3d& dst_normal) {
    return coldet_->CheckCollisionPair(nameA, nameB, dst_point, dst_normal);
  }

  /// @brief Ray casting function
  /// @param [in] start_point Starting point
  /// @param [in] direction Direction vector
  /// @param [in] length Length of ray
  /// @param [out] dst_end_point Projection point
  /// @param [out] dst_name Projected object name
  /// @return bool true if projection point exists
  bool RayCasting(const Eigen::Vector3d& start_point,
                  const Eigen::Vector3d& direction,
                  double length,
                  Eigen::Vector3d& dst_end_point,
                  std::string& dst_name) {
    return coldet_->RayCasting(start_point, direction,
                               length, dst_end_point, dst_name);
  }

  /// @brief Retrieve AABB of object
  /// @param [in] object_name Object name
  /// @return Eigen::Matrix<double, 3, 2>
  ///         AABB of object [xmin xmax; ymin ymax; zmin zmax]
  /// @par Behavior:
  /// - Check if object_name exists in the external object list.
  ///   If it exists, retrieve the overall AABB of the object.
  /// - If object_name does not exist in the external object list,
  ///   retrieve the AABB from the interference check space.
  /// - If object_name does not exist in the interference check space, throw an exception.
  /// @attention
  /// - AABB of child objects can also be retrieved.
  tmc_manipulation_types::AABB GetObjectAABB(
      const std::string& object_name) const;

  /// @brief Retrieve AABB of the entire robot
  /// @return Eigen::Matrix<double, 3, 2>
  ///         AABB of robot [xmin xmax; ymin ymax; zmin zmax]
  /// @par Behavior:
  /// - Retrieve list of robot part names.
  /// - Derive the overall AABB of the robot from each AABB.
  tmc_manipulation_types::AABB GetRobotAABB() const;

  /// @brief Enable only overlapping Cuboids with the robot
  /// @param [in] overlap_type Determine "overlapping" by XY plane or AABB
  /// @param [in] group_type Perform "overlapping" for the entire robot
  ///                        or for each part group
  /// @par Behavior:
  /// - Retrieve robot's AABB according to group_type.
  ///   If group_type is an incorrect value, throw an exception.
  /// - For Cuboids with enabled interference checks, determine
  ///   overlap according to overlap_type. If not overlapping, disable interference check.
  /// - For Cuboids overlapping with AABB obtained above in x-axis direction,
  ///   determine overlap according to overlap_type.
  ///   If overlapping, enable interference check.
  void RefleshOverlappedCuboids(CuboidOverlapType overlap_type,
                                CuboidOverlapGroupType group_type);

  /// @brief Enable Cuboids belonging to the group for interference checks
  /// @param [in] group_name Group name to enable
  /// @par Behavior
  /// - If group_name is an incorrect value, throw an exception
  /// - Enable Cuboids belonging to the group for interference checks
  void EnableCuboids(const std::string& group_name);

  /// @brief Disable Cuboids belonging to the group for interference checks
  /// @param [in] group_name Group name to disable
  /// @par Behavior
  /// - If group_name is an incorrect value, throw an exception
  /// - Disable Cuboids belonging to the group for interference checks
  void DisableCuboids(const std::string& group_name);

  /// @brief Return number of enabled Cuboids, for test/evaluation
  /// @return uint32_t Number of enabled Cuboids
  uint32_t GetEnableCuboidsNum();

  /// @brief Retrieve names of all objects in the interference check space
  /// @return std::vector<std::string> Names of all objects
  std::vector<std::string> GetObjectNameList() const;

  /// @brief Retrieve names of all objects belonging to the group
  /// @param [in] group_name Group name
  /// @return std::vector<std::string> Names of objects in the group
  /// @note
  /// - Grasped objects will belong to the group of the part grasping them
  /// - Each Cuboid will represent its own group
  std::vector<std::string> GetObjectNameListByGroup(
      const std::string& group_name) const;

  /// @brief Retrieve list of group names
  /// @return std::vector<std::string> List of group names
  std::vector<std::string> GetGroupList() const {
    return robot_collision_config_->GetGroupNameList();
  }

  /// @brief Retrieve list of grasped object names
  /// @return std::vector<std::string> List of grasped object names
  std::vector<std::string> GetHeldObjectList() const;

 protected:
  /// @brief Set object's attributes
  /// @param [in] object_name Name of the object to set
  /// @param [in] set_property_function Setting function
  void SetObjectProperty_(
      const std::string& object_name,
      std::function<void(const std::string&)> set_property_function);

  /// @brief Check if it is an internal object
  /// @param [in] Object name
  /// @return bool true if internal object
  /// @note
  /// - Internal objects refer to robot parts and grasped objects
  bool IsInnerObject_(const std::string& object_name);

  /// @brief Check if it is a child object
  /// @param [in] Object name
  /// @return bool true if child object
  bool IsChildObject_(const std::string& object_name);

  /// @brief Update interference check model for nearby check of internal objects
  void UpdateInnerModel_(void);

  /// @brief Update interference check model based on robot kinematic model
  void UpdateCollisionModel_(void);

  /// @brief Create object-object name pair list
  /// @param [in] object_name1 Object name
  /// @param [in] object_name2 Object name
  /// @return std::vector<PairString> Combination of objects
  std::vector<PairString> GetObjectToObjectPairNameList_(
      const std::string& object_name1, const std::string& object_name2);

  /// @brief Create object-group name pair list
  /// @param [in] object_name Object name
  /// @param [in] group_name Group name
  /// @return std::vector<PairString> Combination of objects
  std::vector<PairString> GetObjectToGroupPairNameList_(
      const std::string& object_name, const std::string& group_name);

  /// @brief Retrieve child object names from object name
  /// @param [in] object_name Object name
  /// @return List of object names
  std::vector<std::string> GetChildObjectNameList_(
      const std::string& object_name);

  /// @brief Remove from bounding_box_
  /// @param [in] box_name Name of the Cuboid to remove
  void EraseCuboid_(const std::string& box_name);

  /// @brief Retrieve robot part information from file
  /// @param [in] robot_model_config Path to the robot model configuration file
  /// @param [in] model_file_type Type of the robot model
  /// @return std::vector<tmc_manipulation_types::ObjectParameter>
  ///         Shape information of the robot parts
  tmc_manipulation_types::ObjectParameterSeq GetRobotPartsShape_(
      const std::string& robot_model_config,
      ModelFileType model_file_type);

  /// @brief Load robot model and create robot
  /// @param [in] model_config Path to the model file
  /// @param [in] model_file_type Type of the robot model
  void CreateRobotModel_(const std::string& model_config,
                         ModelFileType model_file_type);

  /// @brief Subtask of constructor
  /// @param [in] robot_model_config Path to the robot model configuration file
  /// @param [in] robot_collision_config Path to the interference check configuration file
  /// @param [in] engine Name of the engine used for interference check
  /// @param [in] model_file_type Type of the robot model
  /// @par Behavior:
  /// - Create a robot kinematic model.
  /// - Load interference check settings.
  /// - Create interference check space.
  /// - If any of these fail, an exception is thrown.
  void Init_(const std::string& robot_model_config,
             const std::string& robot_collision_config,
             const std::string& engine,
             ModelFileType model_file_type);

  /// @brief Retrieve position from kinematic model, a function to absorb difference in handling collision information of URDF
  /// @param [in] object_name Object name
  /// @return Eigen::Affine3d Object posture
  Eigen::Affine3d GetObjectTransformFromKinematicsModel_(const std::string& object_name) const;

  /// Interference check (both internal and external)
  tmc_collision_detector::ICollisionDetector::Ptr coldet_;
  /// Interference check (for internal)
  tmc_collision_detector::ICollisionDetector::Ptr inner_coldet_;
  /// Robot forward kinematic model
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_model_;
  /// Interference check settings
  CollisionDetectorConfig::Ptr robot_collision_config_;
  /// List of names of robot parts
  std::vector<std::string> robot_parts_name_;
  /// Name and group of grasped objects
  std::map<std::string, std::string> attached_object_name_;
  /// List of external objects
  OuterObjectMap outer_object_list_;
  /// Structure for saving environment information
  DumpEnvironmentalData environmental_data_;
  /// Cuboid
  BoundingBox bounding_box_;
  /// Information for position/posture of collision
  std::map<std::string, CollisionFrameInfo> collision_frame_info_map_;
};
}  // namespace tmc_robot_collision_detector
#endif  // TMC_ROBOT_COLLISION_DETECTOR_ROBOT_COLLISION_DETECTOR_HPP_
