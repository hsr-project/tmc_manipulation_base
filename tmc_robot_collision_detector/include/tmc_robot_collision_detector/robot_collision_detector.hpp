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
/// @file robot_collision_detector.hpp
/// @brief Perform interference checks using the robot model
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
  /// @param [in] engine Name of the engine used for interference checks
  /// @par Behavior:
  /// - Create the robot kinematic model.
  /// - Load the interference check configuration.
  /// - Create the interference check space.
  /// - In each case, throw an exception if it fails.
  RobotCollisionDetector(const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot_model,
                         const std::string& robot_model_config,
                         const std::string& robot_collision_config,
                         const std::string& engine);

  /// @brief Constructor. Performs various initializations
  /// @param [in] robot_model_config Path to the robot model configuration file
  /// @param [in] robot_collision_config Path to the interference check configuration file
  /// @param [in] engine Name of the engine used for interference checks
  /// @par Behavior:
  /// - Create the robot kinematic model.
  /// - Load the interference check configuration.
  /// - Create the interference check space.
  /// - In each case, throw an exception if it fails.
  RobotCollisionDetector(const std::string& robot_model_config,
                         const std::string& robot_collision_config,
                         const std::string& engine);

  /// @brief Constructor. Performs various initializations
  /// @param [in] robot_model_config Path to the robot model configuration file
  /// @param [in] robot_collision_config Path to the interference check configuration file
  /// @param [in] engine Name of the engine used for interference checks
  /// @param [in] model_file_type Type of the robot model
  /// @par Behavior:
  /// - Create the robot kinematic model.
  /// - Load the interference check configuration.
  /// - Create the interference check space.
  /// - In each case, throw an exception if it fails.
  RobotCollisionDetector(const std::string& robot_model_config,
                         const std::string& robot_collision_config,
                         const std::string& engine,
                         ModelFileType model_file_type);

  virtual ~RobotCollisionDetector() {}

  /// @brief Input joint angle information
  /// @param [in] named_angle Joint angle information
  /// @par Behavior:
  /// - Pass named_angle to the robot kinematic model.
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

  /// @brief Retrieve the Min and Max of a joint
  /// @param [in] use_joints Names of the joints to retrieve
  /// @param [out] min Lower limit of joint angles
  /// @param [out] max Upper limit of joint angles
  void GetRobotAngleMinMax(
      const tmc_manipulation_types::NameSeq& use_joints,
      Eigen::VectorXd& min,
      Eigen::VectorXd& max) const {
    robot_model_->GetMinMax(use_joints, min, max);
  }

  /// @brief Input the robot's position and orientation
  /// @param [in] Eigen::Affine3d Robot's position and orientation
  /// @par Behavior:
  /// - Retrieve the robot's position and orientation from the robot kinematic model.
  void SetRobotTransform(const Eigen::Affine3d& origin_to_robot);

  /// @brief Retrieve the robot's position and orientation
  /// @return Eigen::Affine3d Robot's position and orientation
  /// @par Behavior:
  /// - Retrieve the robot's position and orientation from the robot kinematic model.
  Eigen::Affine3d GetRobotTransform(void) const {
    return robot_model_->GetRobotTransform();
  }

  /// @brief Create external objects
  /// @param [in] parameters Information about the external object
  /// @par Behavior:
  /// - Verify that the number of child object poses matches the number of shape information.
  ///   If they differ, throw an exception.
  /// - Verify that the object name is set; if not, throw an exception.
  /// - Check if an object with the same name has already been created.
  ///   If it already exists, throw an exception.
  /// - Generate the object in the interference check space and
  ///   add it to the external object list.
  /// @attention
  /// - The created object becomes subject to interference checks.
  /// - The names of child objects are composed of the parent object name + # + serial number.
  void CreateOuterObject(
      const tmc_manipulation_types::OuterObjectParameters& parameters);

  /// @brief Construct an environment composed of Cuboids
  /// @param [in] map Vector of Cuboids
  /// @param [in] enable_flag Enable/disable flag for interference checks when creating Cuboids
  ///             true=enabled, false=disabled
  /// @par Behavior:
  /// - Create Cuboids in the interference check space according to the map.
  ///   If the Cuboid name is empty, throw an exception.
  /// - Disable interference checks for the created Cuboids.
  /// - Add to the external object list.
  /// - Add to the Cuboid list.
  /// @attention
  /// - If you simply want to perform interference checks, it is recommended to set enable_flag to false.
  /// - The group of Cuboids will be CUBOID.
  void CreateCuboids(const tmc_manipulation_types::CuboidSeq& map,
                     bool enable_flag);

  /// @brief Construct an environment composed of Cuboids
  /// @param [in] map Vector of Cuboids
  /// @param [in] enable_flag Enable/disable flag for interference checks when creating Cuboids
  ///             true=enabled, false=disabled
  /// @param [in] cuboid_group_name Group name of the Cuboids
  /// @par Behavior:
  /// - Create Cuboids in the interference check space according to the map.
  ///   If the Cuboid name is empty, throw an exception.
  /// - Disable interference checks for the created Cuboids.
  /// - Add to the external object list.
  /// - Add to the Cuboid list.
  /// @attention
  /// - If you simply want to perform interference checks, it is recommended to set enable_flag to false.
  void CreateCuboids(const tmc_manipulation_types::CuboidSeq& map,
                     bool enable_flag,
                     const std::string& cuboid_group_name);

  /// @brief Discard external objects
  /// @param [in] object_name Name of the object to discard
  /// @par Behavior:
  /// - Check if object_name is a robot part.
  ///   If it is a robot part, do nothing.
  /// - Check if object_name is a grasped object.
  ///   If it is a grasped object, do nothing.
  /// - Check if object_name exists in the external object list.
  ///   If it does not exist, throw an exception.
  /// - Discard the object from the interference check space.
  /// @attention
  /// - Only child objects cannot be discarded.
  /// - Cuboids can be discarded.
  void DestroyOuterObject(const std::string& object_name);

  /// @brief Discard all external objects
  /// @par Behavior:
  /// - Discard all external objects from the interference check space.
  /// @attention
  /// - Grasped objects will also be discarded.
  /// - All Cuboids will also be discarded.
  void DestroyAllOuterObject(void);

  /// @brief Discard the environment composed of Cuboids
  /// @par Behavior:
  /// - Discard Cuboids existing in the Cuboid list from the interference check space.
  /// - Discard the Cuboid list.
  void DestroyCuboids();

  /// @brief Retrieve information about an object
  /// @param [in] object_name Name of the object
  /// @return OuterObjectParameters Information about the object
  /// @par Behavior:
  /// - If it is an external object, retrieve and return information from the external object list.
  /// - Otherwise, retrieve and return information from the interference check space.
  /// - If object_name does not exist, throw an exception.
  tmc_manipulation_types::OuterObjectParameters GetObjectParameter(
      const std::string& object_name) const;

  /// @brief Retrieve information about all external objects
  /// @return OuterObjectParametersSeq Information about all external objects
  /// @par Behavior:
  /// - Return the external object list.
  tmc_manipulation_types::OuterObjectParametersSeq
  GetAllOuterObjectParameters(void) const;

  /// @brief Enable interference checks for an object
  /// @param [in] object_name Name of the object
  /// @par Behavior:
  /// - Enable interference checks for the object in the interference check space.
  /// @attention
  /// - If a parent object is specified, all child objects are targeted.
  /// - If a child object is specified, only the child object is targeted.
  /// - If a Cuboid is specified, it is removed from the Cuboid list and
  ///   becomes a simple external object.
  void EnableCollisionObject(const std::string& object_name);

  /// @brief Disable interference checks for an object
  /// @param [in] object_name Name of the object
  /// @par Behavior:
  /// - Disable interference checks for the object in the interference check space.
  /// @attention
  /// - If a parent object is specified, all child objects are targeted.
  /// - If a child object is specified, only the child object is targeted.
  /// - If a Cuboid is specified, it is removed from the Cuboid list and
  ///   becomes a simple external object.
  void DisableCollisionObject(const std::string& object_name);

  /// @brief Change the group of an object
  /// @param [in] object_name Name of the object
  /// @param [in] category Group bit to change to
  /// @par Behavior:
  /// - Change the group of the object in the interference check space.
  /// @attention
  /// - If a parent object is specified, all child objects are targeted.
  /// - If a child object is specified, only the child object is targeted.
  /// - If a Cuboid is specified, it is removed from the Cuboid list and
  ///   becomes a simple external object.
  void SetObjectGroup(const std::string& object_name, uint16_t category);

  /// @brief Reset the group of objects to default
  /// @param [in] object_name Object name
  /// @par Behavior:
  /// - Retrieve the default group bit from the interference check settings
  /// - Change the group of objects in the interference check space.
  /// @attention
  /// - If a parent object is specified, all child objects will be targeted.
  /// - If a child object is specified, only the child object will be targeted.
  /// - If a cuboid is specified, it will be removed from the cuboid list,
  ///   becoming just an external object.
  void SetObjectDefaultGroup(const std::string& object_name);

  /// @brief Retrieve the group of an object
  /// @param [in] object_name Object name
  /// @return uint16_t Group bit
  /// @par Behavior:
  /// - Retrieve the current group bit from the interference check space
  /// @attention
  /// - If a parent object is specified, the first child object will be targeted.
  uint16_t GetObjectGroup(const std::string& object_name) const;

  /// @brief Retrieve the default group of an object
  /// @param [in] object_name Object name
  /// @return uint16_t Group bit
  /// @par Behavior:
  /// - Retrieve the default group bit from the interference check settings
  uint16_t GetObjectDefaultGroup(const std::string& object_name) const;

  /// @brief Change the filter of an object
  /// @param [in] object_name Object name
  /// @param [in] filter Filter bit to be changed
  /// @par Behavior:
  /// - Change the filter of objects in the interference check space.
  /// @attention
  /// - If a parent object is specified, all child objects will be targeted.
  /// - If a child object is specified, only the child object will be targeted.
  /// - If a cuboid is specified, it will be removed from the cuboid list,
  ///   becoming just an external object.
  void SetObjectFilter(const std::string& object_name, uint16_t filter);

  /// @brief Reset the filter of an object to default
  /// @param [in] object_name Object name
  /// @par Behavior:
  /// - Retrieve the default filter bit from the interference check settings
  /// - Change the filter of objects in the interference check space.
  /// @attention
  /// - If a parent object is specified, all child objects will be targeted.
  /// - If a child object is specified, only the child object will be targeted.
  /// - If a cuboid is specified, it will be removed from the cuboid list,
  ///   becoming just an external object.
  void SetObjectDefaultFilter(const std::string& object_name);

  /// @brief Retrieve the filter of an object
  /// @param [in] object_name Object name
  /// @return uint16_t Filter bit
  /// @par Behavior:
  /// - Retrieve the current filter bit from the interference check space
  /// @attention
  /// - If a parent object is specified, the first child object will be targeted.
  uint16_t GetObjectFilter(const std::string& object_name) const;

  /// @brief Retrieve the default filter of an object
  /// @param [in] object_name Object name
  /// @return uint16_t Filter bit
  /// @par Behavior:
  /// - Retrieve the default filter bit from the interference check settings
  uint16_t GetObjectDefaultFilter(const std::string& object_name) const;

  /// @brief Add a pair of objects to be excluded from interference checks
  /// @param [in] object_name1 Object name
  /// @param [in] object_name2 Object name
  /// @par Behavior:
  /// - Add/remove pairs to/from the exclusion pair list
  ///   and the additional pair list held by the interference check class.
  /// @attention
  /// - To cancel Enable, specify the same pair with Disable.
  /// - If a parent object is specified, all child objects will be targeted.
  /// - If a child object is specified, only the child object will be targeted.
  /// - If a cuboid is specified, it will be removed from the cuboid list,
  ///   becoming just an external object.
  void DisableCollisionCheckObjectToObject(const std::string& object_name1,
                                           const std::string& object_name2);

  /// @brief Add a pair of objects to be included in interference checks
  /// @param [in] object_name1 Object name
  /// @param [in] object_name2 Object name
  /// @par Behavior:
  /// - Add/remove pairs to/from the exclusion pair list
  ///   and the additional pair list held by the interference check class.
  /// @attention
  /// - To cancel Enable, specify the same pair with Disable.
  /// - If a parent object is specified, all child objects will be targeted.
  /// - If a child object is specified, only the child object will be targeted.
  /// - If a cuboid is specified, it will be removed from the cuboid list,
  ///   becoming just an external object.
  void EnableCollisionCheckObjectToObject(const std::string& object_name1,
                                          const std::string& object_name2);

  /// @brief Add a pair of object-group to be excluded from interference checks
  /// @param [in] object_name Object name
  /// @param [in] group_name Group name
  /// @par Behavior:
  /// - Add/remove pairs to/from the exclusion pair list
  ///   and the additional pair list held by the interference check class.
  /// @attention
  /// - To cancel Enable, specify the same pair with Disable.
  /// - If a parent object is specified, all child objects will be targeted.
  /// - If a child object is specified, only the child object will be targeted.
  /// - If a cuboid is specified, it will be removed from the cuboid list,
  ///   becoming just an external object.
  void DisableCollisionCheckObjectToGroup(const std::string& object_name,
                                          const std::string& group_name);

  /// @brief Add a pair of object-group to be included in interference checks
  /// @param [in] object_name Object name
  /// @param [in] group_name Group name
  /// @par Behavior:
  /// - Add/remove pairs to/from the exclusion pair list
  ///   and the additional pair list held by the interference check class.
  /// @attention
  /// - To cancel Enable, specify the same pair with Disable.
  /// - If a parent object is specified, all child objects will be targeted.
  /// - If a child object is specified, only the child object will be targeted.
  /// - If a cuboid is specified, it will be removed from the cuboid list,
  ///   becoming just an external object.
  void EnableCollisionCheckObjectToGroup(const std::string& object_name,
                                         const std::string& group_name);

  /// @brief Change the filter of objects belonging to each group
  ///        to exclude them from interference checks
  /// @param [in] group_name1 Group name
  /// @param [in] group_name2 Group name
  /// @par Behavior:
  /// - Retrieve the list of objects belonging to each group.
  /// - Manipulate the filter of objects to prevent interference checks.
  void DisableCollisionCheckGroupToGroup(const std::string& group_name1,
                                         const std::string& group_name2);

  /// @brief Change the filter of objects belonging to each
  ///        group to include them in interference checks
  /// @param [in] group_name1 Group name
  /// @param [in] group_name2 Group name
  /// @par Behavior:
  /// - Retrieve the list of objects belonging to each group.
  /// - Manipulate the filter of objects to enable interference checks.
  void EnableCollisionCheckGroupToGroup(const std::string& group_name1,
                                        const std::string& group_name2);

  /// @brief Set the position and orientation of an object
  /// @param [in] object_name Object name to be set
  /// @param [in] origin_to_object Position and orientation to be set
  /// @par Behavior:
  /// - Check if object_name is a robot part.
  ///   If it is a robot part, do nothing.
  /// - Check if object_name is a grasped object.
  ///   If it is a grasped object, do nothing.
  /// - Check if object_name exists in the external object list.
  ///   If it does not exist, throw an exception.
  /// - Change the position and orientation of the object in the interference check space.
  /// - Update the posture information of the object.
  /// @attention
  /// - The posture of child objects cannot be changed.
  ///   They follow the changes of the parent object.
  /// - If a cuboid is specified, it will be removed from the cuboid list,
  ///   becoming just an external object.
  void SetObjectTransform(const std::string& object_name,
                          const Eigen::Affine3d& origin_to_object);

  /// @brief Retrieve the position of an object
  /// @param [in] object_name Object name
  /// @return Eigen::Affine3d Object posture
  /// @par Behavior:
  /// - Check if object_name exists in the external object list.
  ///   If it exists, return the posture of the object from the object information.
  /// - Retrieve the position and orientation of the object from the interference check space.
  ///   If it can be retrieved, return the posture.
  /// - Retrieve the position and orientation of the object from the robot model.
  ///   If it can be retrieved, return the posture.
  /// - If it cannot be retrieved from any of the above, throw an exception.
  /// @attention
  /// - The posture of child objects, parts (joints, etc.) that exist only in the robot model,
  ///   and the posture of cuboids can also be retrieved.
  Eigen::Affine3d GetObjectTransform(const std::string& object_name) const;

  /// @brief Grasp an object
  /// @param [in] object_name Object name to be grasped
  /// @param [in] frame_name Object name
  /// @param [in] held_frame_to_object
  ///             Relative position of the object to be grasped based on the grasping object
  /// @param [in] held_group_name Group name of the grasping object
  /// @par Behavior:
  /// - Check if object_name is already being grasped. If it is, do nothing.
  /// - Check if object_name is a robot part.
  ///   If it is a robot part, do nothing.
  /// - Check if object_name exists in the external object list.
  ///   If it does not exist, throw an exception.
  /// - According to the group name of the grasping object,
  ///   change the group/filter of the object to be grasped.
  /// - Add frame_name, held_frame_to_object,
  ///   and held_group_name to the object information.
  /// - Update the position of the object to be grasped in the interference check space.
  /// - Update the list of grasped objects.
  /// @attention
  /// - Only child objects cannot be grasped.
  /// - If a cuboid is specified, it will be removed from the cuboid list,
  ///   becoming a grasped object.
  void HoldObject(const std::string& object_name,
                  const std::string& frame_name,
                  const Eigen::Affine3d& held_frame_to_object,
                  const std::string& held_group_name);

  /// @brief Grasp an object, the group of the grasping object becomes the same as frame_name
  /// @param [in] object_name Object name to be grasped
  /// @param [in] frame_name Object name
  /// @param [in] held_frame_to_object
  ///             Relative position of the object to be grasped based on the grasping object
  /// @par Behavior:
  /// - Retrieve the group name from frame_name.
  void HoldObject(const std::string& object_name,
                  const std::string& frame_name,
                  const Eigen::Affine3d& held_frame_to_object);

  /// @brief Release a grasped object
  /// @param [in] object_name Object name to be released
  /// @par Behavior:
  /// - Update the list of grasped objects.
  ///   If object_name does not exist in the list of grasped objects, throw an exception.
  /// - Reset the category/filter of the grasped object to default.
  void ReleaseObject(const std::string& object_name);

  /// @brief Release all grasped objects
  /// @par Behavior:
  /// - Release all grasped objects.
  void ReleaseAllObject(void);

  /// @brief Interference check, terminate the check if interference is detected
  /// @return bool True if interference is detected
  virtual bool CheckCollision();

  /// @brief Interference check, output the names of interfering objects
  /// @param [in] end_flag If false, perform interference check until the end
  ///             and create an interference pair list
  /// @param [out] dst_contact_pair Interference pair
  /// @return bool True if interference is detected
  bool CheckCollision(bool end_flag,
                      std::vector<PairString>& dst_contact_pair);

  /// @brief Search for nearby objects for each robot part
  /// @param [in] extend_length Maximum distance to nearby objects
  /// @param [in] top_n Number of objects for which accurate distance is calculated using the GJK algorithm
  /// @param [out] dst_result_list Information on nearby objects for each robot part
  /// @return bool True if interference is detected
  /// @par Behavior:
  /// - Determine nearby objects for internal/external objects
  ///   for robot parts and grasped objects.
  /// - Details of the search are described in the collision detector README.
  bool CheckClosestObject(double extend_length, int32_t top_n,
                          std::vector<ClosestObject>& dst_result_list);

  /// @brief Search for nearby objects of object_name
  /// @param [in] object_name Object name to be searched
  /// @param [in] extend_length Maximum distance to nearby objects
  /// @param [in] top_n Number of objects for which accurate distance is calculated using the GJK algorithm
  /// @param [out] dst_result Information on nearby objects for each robot part
  /// @return bool True if interference is detected
  /// @par Behavior:
  /// - Details of the search are described in the collision detector README.
  bool CheckClosestObject(const std::string& object_name, double extend_length,
                          int32_t top_n, ClosestObject& dst_result);

  /// @brief Check if two objects are interfering
  /// @param [in] nameA Object A
  /// @param [in] nameB Object B
  /// @param [in] dst_point Contact point between A and B
  /// @param [in] dst_normal Normal at the contact point between A and B
  /// @return bool True if interference exists
  bool CheckCollisionPair(
      const std::string& nameA, const std::string& nameB,
      Eigen::Vector3d& dst_point, Eigen::Vector3d& dst_normal) {
    return coldet_->CheckCollisionPair(nameA, nameB, dst_point, dst_normal);
  }

  /// @brief Check if two objects are interfering
  /// @param [in] nameA Object A
  /// @param [in] nameB Object B
  /// @param [out] depth Depth of interference
  /// @return bool True if interference exists
  bool CheckCollisionPair(
      const std::string& nameA, const std::string& nameB, double& depth) {
    return coldet_->CheckCollisionPair(nameA, nameB, depth);
  }

  /// @brief Ray casting function
  /// @param [in] start_point Starting point
  /// @param [in] direction Direction vector
  /// @param [in] length Ray length
  /// @param [out] dst_end_point Projected point
  /// @param [out] dst_name Name of the projected object
  /// @return bool True if a projected point exists
  bool RayCasting(const Eigen::Vector3d& start_point,
                  const Eigen::Vector3d& direction,
                  double length,
                  Eigen::Vector3d& dst_end_point,
                  std::string& dst_name) {
    return coldet_->RayCasting(start_point, direction,
                               length, dst_end_point, dst_name);
  }

  /// @brief Retrieve the AABB of an object
  /// @param [in] object_name Object name
  /// @return Eigen::Matrix<double, 3, 2>
  ///         AABB of the object [xmin xmax; ymin ymax; zmin zmax]
  /// @par Behavior:
  /// - Check if object_name exists in the external object list.
  ///   If it exists, retrieve the entire AABB of the object.
  /// - If object_name does not exist in the external object list,
  ///   retrieve the AABB from the interference check space.
  /// - If object_name does not exist in the interference check space, throw an exception.
  /// @attention
  /// - The AABB of child objects can also be retrieved.
  tmc_manipulation_types::AABB GetObjectAABB(
      const std::string& object_name) const;

  /// @brief Retrieve the AABB of the entire robot
  /// @return Eigen::Matrix<double, 3, 2>
  ///         AABB of the robot [xmin xmax; ymin ymax; zmin zmax]
  /// @par Behavior:
  /// - Retrieve the list of robot part names.
  /// - Derive the entire robot's AABB from each AABB.
  tmc_manipulation_types::AABB GetRobotAABB() const;

  /// @brief Enable only the cuboids overlapping with the robot
  /// @param [in] overlap_type Determine "overlapping" by XY plane or AABB
  /// @param [in] group_type Determine "overlapping" for the entire robot
  ///                        or for each part group
  /// @par Behavior:
  /// - Retrieve the robot's AABB according to group_type.
  ///   If group_type has an invalid value, throw an exception.
  /// - For cuboids with interference checks enabled, determine overlapping
  ///   according to overlap_type. If not overlapping, disable interference checks.
  /// - For cuboids overlapping with the AABB obtained above in the x-axis direction,
  ///   determine overlapping according to overlap_type.
  ///   If overlapping, enable interference checks.
  void RefleshOverlappedCuboids(CuboidOverlapType overlap_type,
                                CuboidOverlapGroupType group_type);

  /// @brief Enable cuboids belonging to a group for interference checks
  /// @param [in] group_name Group name to be enabled
  /// @par Behavior
  /// - If group_name has an invalid value, throw an exception
  /// - Enable cuboids belonging to group_name for interference checks
  void EnableCuboids(const std::string& group_name);

  /// @brief Disable cuboids belonging to a group for interference checks
  /// @param [in] group_name Group name to be disabled
  /// @par Behavior
  /// - If group_name has an invalid value, throw an exception
  /// - Disable cuboids belonging to group_name for interference checks
  void DisableCuboids(const std::string& group_name);

  /// @brief Return the number of enabled cuboids, for testing/evaluation
  /// @return uint32_t Number of enabled cuboids
  uint32_t GetEnableCuboidsNum();

  /// @brief Retrieve the names of all objects in the interference check space
  /// @return std::vector<std::string> Names of all objects
  std::vector<std::string> GetObjectNameList() const;

  /// @brief Retrieve the names of all objects belonging to a group
  /// @param [in] group_name Group name
  /// @return std::vector<std::string> Names of objects belonging to the group
  /// @note
  /// - Grasped objects belong to the group of the part grasping them
  /// - Cuboids belong to their respective groups
  std::vector<std::string> GetObjectNameListByGroup(
      const std::string& group_name) const;

  /// @brief Retrieve the list of group names
  /// @return std::vector<std::string> List of group names
  std::vector<std::string> GetGroupList() const {
    return robot_collision_config_->GetGroupNameList();
  }

  /// @brief Retrieve the list of names of grasped objects
  /// @return std::vector<std::string> List of names of grasped objects
  std::vector<std::string> GetHeldObjectList() const;

 protected:
  /// @brief Set the attributes of an object
  /// @param [in] object_name Object name to be set
  /// @param [in] set_property_function Function to be set
  void SetObjectProperty_(
      const std::string& object_name,
      std::function<void(const std::string&)> set_property_function);

  /// @brief Check if it is an internal object
  /// @param [in] Object name
  /// @return bool True if it is an internal object
  /// @note
  /// - Internal objects refer to robot parts and grasped objects
  bool IsInnerObject_(const std::string& object_name);

  /// @brief Check if it is a child object
  /// @param [in] Object name
  /// @return bool True if it is a child object
  bool IsChildObject_(const std::string& object_name);

  /// @brief Update the interference check model for nearby checks of internal objects
  void UpdateInnerModel_(void);

  /// @brief Update the interference check model based on the robot's kinematic model
  void UpdateCollisionModel_(void);

  /// @brief Create a list of object-object name pairs
  /// @param [in] object_name1 Object name
  /// @param [in] object_name2 Object name
  /// @return std::vector<PairString> Object combinations
  std::vector<PairString> GetObjectToObjectPairNameList_(
      const std::string& object_name1, const std::string& object_name2);

  /// @brief Create object-group name pairs
  /// @param [in] object_name Object name
  /// @param [in] group_name Group name
  /// @return std::vector<PairString> Object combinations
  std::vector<PairString> GetObjectToGroupPairNameList_(
      const std::string& object_name, const std::string& group_name);

  /// @brief Retrieve child object names from an object name
  /// @param [in] object_name Object name
  /// @return List of object names
  std::vector<std::string> GetChildObjectNameList_(
      const std::string& object_name);

  /// @brief Remove from bounding_box_
  /// @param [in] box_name Name of the cuboid to be removed
  void EraseCuboid_(const std::string& box_name);

  /// @brief Retrieve robot part information from a file
  /// @param [in] robot_model_config Path to the robot model configuration file
  /// @param [in] model_file_type Type of robot model
  /// @return std::vector<tmc_manipulation_types::ObjectParameter>
  ///         Shape information of robot parts
  tmc_manipulation_types::ObjectParameterSeq GetRobotPartsShape_(
      const std::string& robot_model_config,
      ModelFileType model_file_type);

  /// @brief Load the robot model and create the robot
  /// @param [in] model_config Path to the model file
  /// @param [in] model_file_type Type of robot model
  void CreateRobotModel_(const std::string& model_config,
                         ModelFileType model_file_type);

  /// @brief Subroutine for the constructor
  /// @param [in] robot_model_config Path to the robot model configuration file
  /// @param [in] robot_collision_config Path to the interference check configuration file
  /// @param [in] engine Name of the engine used for interference checks
  /// @param [in] model_file_type Type of robot model
  /// @par Behavior:
  /// - Create the robot kinematic model.
  /// - Load the interference check settings.
  /// - Create the interference check space.
  /// - If any of the above fails, throw an exception.
  void Init_(const std::string& robot_model_config,
             const std::string& robot_collision_config,
             const std::string& engine,
             ModelFileType model_file_type);

  /// @brief Retrieve position from the kinematic model, a function to absorb differences in handling collision information in URDF
  /// @param [in] object_name Object name
  /// @return Eigen::Affine3d Object posture
  Eigen::Affine3d GetObjectTransformFromKinematicsModel_(const std::string& object_name) const;

  /// Interference check (both internal and external)
  tmc_collision_detector::ICollisionDetector::Ptr coldet_;
  /// Interference check (internal)
  tmc_collision_detector::ICollisionDetector::Ptr inner_coldet_;
  /// Robot forward kinematic model
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_model_;
  /// Interference check settings
  CollisionDetectorConfig::Ptr robot_collision_config_;
  /// List of robot part names
  std::vector<std::string> robot_parts_name_;
  /// Names and groups of grasped objects
  std::map<std::string, std::string> attached_object_name_;
  /// List of external objects
  OuterObjectMap outer_object_list_;
  /// Structure for storing environmental information
  DumpEnvironmentalData environmental_data_;
  /// Cuboid
  BoundingBox bounding_box_;
  /// Information for collision position/orientation
  std::map<std::string, CollisionFrameInfo> collision_frame_info_map_;
};
}  // namespace tmc_robot_collision_detector
#endif  // TMC_ROBOT_COLLISION_DETECTOR_ROBOT_COLLISION_DETECTOR_HPP_
