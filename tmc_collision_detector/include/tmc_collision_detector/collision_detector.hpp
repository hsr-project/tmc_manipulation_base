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
/// @file     collision_detector.hpp
/// @brief    Interface class for collision detection
/// @author   Keisuke Takeshita
/// @version  1.0.0
/// @date     2012.05.24
#ifndef TMC_COLLISION_DETECTOR_COLLISION_DETECTOR_HPP_
#define TMC_COLLISION_DETECTOR_COLLISION_DETECTOR_HPP_

#include  <stdint.h>
#include  <utility>

#include  <list>
#include  <map>
#include  <memory>
#include  <string>
#include  <vector>

#include  <Eigen/Core>
#include  <Eigen/Geometry>

#include  <tmc_manipulation_types/manipulation_types.hpp>
#include  "tmc_collision_detector/collision_detector_exception.hpp"

namespace tmc_collision_detector {
using PairString = std::pair<std::string, std::string>;

/// Structure to store the nearest object, nearby point, and distance
struct ClosestResult {
  std::string name;  /// Name of the nearby object
  bool contact;  /// True if in contact
  double distance;  /// Distance between objects
  Eigen::Vector3d closest_point_on_A;  /// Nearby point on the object
  Eigen::Vector3d closest_point_on_B;  /// Nearby point on the nearby object
  Eigen::Vector3d normal_on_B;  // Normal on B
};
/// Collision check interface class
class ICollisionDetector {
 public:
  using Ptr = std::shared_ptr<ICollisionDetector>;

  virtual ~ICollisionDetector() {}

  /// @brief Create an object
  /// @param  [in,out] parameter Information about the object
  /// @par Behavior:
  /// - Check if parameter.shape.dimensions are non-negative. If negative, throw the exception InvalidShapeParamError.
  /// - Create primitives based on parameter.shape.type.
  /// - For meshes, if the STL file cannot be loaded, throw the exception InvalidShapeParamError.
  /// - If type is not one of the values of CollisionObjectType, throw the exception NonExistTypeError.
  /// - Set the position and orientation according to parameter.transform.
  /// - Set the object's filter/group according to parameter.filter and parameter.group.
  virtual void CreateObject(
      const tmc_manipulation_types::ObjectParameter& parameter) = 0;

  /// @brief Retrieve object information
  /// @param  [in] name Name of the object to retrieve
  /// @return ObjectParameter Information about the object
  /// @par Behavior:
  /// - Search the list for the object with the name name. If not found, throw the exception NonCreateError.
  /// - Return the object's information.
  virtual tmc_manipulation_types::ObjectParameter GetObjectParameter(
      const std::string& name) const = 0;

  /// @brief Retrieve the object's AABB
  /// @param  [in] name Name of the object to retrieve
  /// @return AABB The object's AABB [xmin xmax; ymin ymax; zmin zmax]
  /// @par Behavior:
  /// - Search the list for the object with the name name. If not found, throw the exception NonCreateError.
  /// - Retrieve the AABB.
  /// - Store the retrieved AABB and return it.
  virtual tmc_manipulation_types::AABB GetObjectAABB(
      const std::string& name) const = 0;


  /// @brief Destroy an object
  /// @param  [in] name Name of the object to destroy
  /// @par Behavior:
  /// - Search the list for the object with the name name. If not found, throw the exception NonCreateError.
  /// - Destroy the object.
  virtual void DestroyObject(const std::string& name) = 0;


  /// @brief Destroy all objects after the anchor (excluding the anchor)
  /// @par Behavior:
  /// - Check if the anchor is set. If not set, throw an exception.
  /// - Sequentially destroy all objects after the anchor.
  /// - If the anchor is the last object, do nothing.
  virtual void DestroyObject(void) = 0;

  /// @brief Set the current last object as the anchor
  /// @par Behavior:
  /// - Check if objects exist in the environment. If none exist, throw an exception.
  /// - Set the last object as the anchor.
  virtual void SetAnchor(void) = 0;

  /// @brief Set the position and orientation of an object
  /// @param  [in] transform Position and orientation to set
  /// @param  [in] name Name of the object to set
  /// @par Behavior:
  /// - Search the list for the object with the name name. If not found, throw the exception NonCreateError.
  /// - Update the object's position and orientation.
  virtual void SetObjectTransform(const Eigen::Affine3d &transform,
                                  const std::string& name) = 0;

  /// @brief Retrieve the position and orientation of an object
  /// @param  [in] name Name of the object to retrieve
  /// @return Eigen::Affine3d Retrieved position and orientation
  /// @par Behavior:
  /// - Search the list for the object with the name name. If not found, throw the exception NonCreateError.
  /// - Return the object's position and orientation.
  virtual Eigen::Affine3d GetObjectTransform(
      const std::string& name) const = 0;

  /// @brief Set the group of an object
  /// @param  [in] group Group bit to set
  /// @param  [in] name Name of the object to set
  /// @par Behavior:
  /// - Search the list for the object with the name name. If not found, throw the exception NonCreateError.
  /// - Set the object's group.
  virtual void SetCollisionGroup(const uint16_t group,
                                 const std::string& name) = 0;

  /// @brief Set the filter of an object
  /// @param  [in] filter Filter bit to set
  /// @param  [in] name Name of the object to set
  /// @par Behavior:
  /// - Search the list for the object with the name name. If not found, throw the exception NonCreateError.
  /// - Set the object's filter.
  virtual void SetCollisionFilter(const uint16_t filter,
                                  const std::string& name) = 0;

  /// @brief Retrieve the group of an object
  /// @param  [in] name Name of the object to retrieve
  /// @return uint16_t Group of the object
  /// @par Behavior:
  /// - Search the list for the object with the name name. If not found, throw the exception NonCreateError.
  /// - Retrieve the object's group.
  virtual uint16_t GetCollisionGroup(const std::string& name) const = 0;

  /// @brief Retrieve the filter of an object
  /// @param  [in] name Name of the object to retrieve
  /// @return uint16_t Filter of the object
  /// @par Behavior:
  /// - Search the list for the object with the name name. If not found, throw the exception NonCreateError.
  /// - Retrieve the object's filter.
  virtual uint16_t GetCollisionFilter(const std::string& name) const = 0;

  /// @brief Enable collision check for an object
  /// @param  [in] name Name of the object to enable collision check
  /// @par Behavior:
  /// - Search the list for the object with the name name. If not found, throw the exception NonCreateError.
  /// - Enable the object.
  virtual void EnableObject(const std::string& name) = 0;

  /// @brief Disable collision check for an object
  /// @param  [in] name Name of the object to disable collision check
  /// @par Behavior:
  /// - Search the list for the object with the name name. If not found, throw the exception NonCreateError.
  /// - Disable the object.
  virtual void DisableObject(const std::string& name) = 0;

  /// @brief Add object pairs to exclude from collision checks in the space
  /// @param  [in] names Pair of object names to exclude from collision checks
  /// @par Behavior:
  /// - Search the list for objects with names nameA and nameB. If not found, throw the exception NonCreateError.
  /// - Search for the object pair in the collision check addition pair list. If found, remove it from the list.
  /// - If not found, add the object pair to the collision check exclusion pair list.
  /// @attention
  /// - Independently of filters or collision check enable/disable, the state can be set to not perform collision checks.
  /// - By calling ResetCollisionCheckPairList() or specifying with EnableCollisionCheck(),
  ///   the object pair can be removed from the collision check exclusion pair list.
  /// - Object pairs already in the collision check exclusion pair list can be added to the list again.
  virtual void DisableCollisionCheck(const std::vector<PairString>& names) = 0;

  /// @brief Add object pairs to perform collision checks in the space
  /// @param  [in] names Pair of object names to add for collision checks
  /// @par Behavior:
  /// - Search the list for objects with names nameA and nameB. If not found, throw the exception NonCreateError.
  /// - Search for the object pair in the collision check exclusion pair list. If found, remove it from the list.
  /// - If not found, add the object pair to the collision check addition pair list.
  /// @attention
  /// - Independently of filters or collision check enable/disable, the state can be set to perform collision checks.
  /// - By calling ResetCollisionCheckPairList() or specifying with DisableCollisionCheck(),
  ///   the object pair can be removed from the collision check addition pair list.
  /// - Object pairs already in the collision check addition pair list can be added to the list again.
  virtual void EnableCollisionCheck(const std::vector<PairString>& names) = 0;

  /// @brief Clear the collision check exclusion pair list and collision check addition pair list.
  /// @par Behavior:
  /// - Clear the collision check exclusion pair list and collision check addition pair list.
  virtual void ResetCollisionCheckPairList() = 0;

  /// @brief Check if two objects are colliding
  /// @param  [in] nameA Object to perform collision check
  /// @param  [in] nameB Object to perform collision check
  /// @return bool Result of the collision check between two objects. True if colliding
  /// @par Behavior:
  /// - Search the list for objects with names nameA and nameB. If not found, throw the exception NonCreateError.
  /// - Perform collision check between objects.
  /// - Return the result.
  /// @attention
  /// - Calculation is performed even if the object is disabled.
  virtual bool CheckCollisionPair(const std::string& nameA,
                                  const std::string& nameB) = 0;

  /// @brief Check if two objects are colliding
  /// @param  [in] nameA Object to perform collision check
  /// @param  [in] nameB Object to perform collision check
  /// @param  [out] point Contact point on object nameB
  /// @param  [out] normal Normal from the contact point on object nameB to object nameA
  /// @return bool Result of the collision check between two objects. True if colliding
  /// @par Behavior:
  /// - Search the list for objects with names nameA and nameB. If not found, throw the exception NonCreateError.
  /// - Perform collision check between objects.
  /// - Return the result.
  /// @attention
  /// - Calculation is performed even if the object is disabled.
  /// - If not colliding, point and normal will contain 0.
  virtual bool CheckCollisionPair(const std::string& nameA,
                                  const std::string& nameB,
                                  Eigen::Vector3d& point,
                                  Eigen::Vector3d& normal) = 0;

  /// @brief Check if two objects are colliding
  /// @param  [in] nameA Object to perform collision check
  /// @param  [in] nameB Object to perform collision check
  /// @param  [out] depth Depth if colliding
  /// @return bool Result of the collision check between two objects. True if colliding
  /// @attention
  /// - Calculation is performed even if the object is disabled.
  /// - If not colliding, depth will contain 0.0.
  virtual bool CheckCollisionPair(const std::string& nameA,
                                  const std::string& nameB,
                                  double& depth) = 0;

  /// @brief Check if objects in the space are colliding
  /// @return bool Result of the collision check for objects. True if colliding
  /// @par Behavior:
  /// - Utilize the functionality of each physics engine to perform collision checks for objects in the space.
  /// - If a collision occurs, terminate the collision check at that point.
  /// - Return the result.
  /// @attention
  /// - If there is one or fewer objects in the space, the collision check result will be false.
  virtual bool CheckCollisionSpace(void) = 0;

  /// @brief Check if objects in the space are colliding and retrieve the names of the colliding object pairs
  /// @param  [out] dst_contact_pair Names of the colliding object pairs
  /// @return bool Result of the collision check for objects. True if colliding
  /// @par Behavior:
  /// - Utilize the functionality of each physics engine to perform collision checks for objects in the space.
  /// - If a collision occurs, terminate the collision check at that point.
  /// - Retrieve the names of the colliding objects. If names are not found, throw the exception InvalidObjectContactError.
  /// - Return the result.
  /// @attention
  /// - If there is one or fewer objects in the space, the collision check result will be false.
  /// - If the collision check result is false, dst_contact_pair will not be updated.
  virtual bool CheckCollisionSpace(PairString& dst_contact_pair) = 0;

  /// @brief Create a list of colliding object pairs
  /// @param  [out] dst_contact_pair List of names of colliding object pairs
  /// @return bool Result of the collision check for objects. True if colliding
  /// @par Behavior:
  /// - Utilize the functionality of each physics engine to perform collision checks for objects in the space.
  /// - If a collision occurs, retrieve the names of the colliding objects.
  ///   If names are not found, throw the exception InvalidObjectContactError.
  /// - After completing the collision check for objects in the space, return the result.
  /// @attention
  /// - If there is one or fewer objects in the space, the collision check result will be false.
  /// - If the collision check result is false, dst_contact_pair will be empty.
  virtual bool GetContactPairList(
      std::vector<PairString>& dst_contact_pair) = 0;

  /// @brief Retrieve the distance between two objects
  /// @param  [in] nameA Object to perform collision check
  /// @param  [in] nameB Object to perform collision check
  /// @return ClosestResult Result of the collision check between two objects.
  /// @par Behavior:
  /// - Search the list for objects with names nameA and nameB. If not found, throw the exception NonCreateError.
  /// - Calculate the distance between objects.
  /// @attention
  /// - Calculation is performed even if the object is disabled.
  virtual ClosestResult GetClosestResult(const std::string& nameA,
                                         const std::string& nameB) = 0;

  /// @brief Retrieve information about the nearest object in the space to a given object
  /// @param  [in] name Name of the object for which to retrieve nearby objects
  /// @param  [in] extend_length Upper limit of the distance to nearby objects
  /// @param  [in] top_n Number of objects for which the GJK algorithm calculates accurate distances
  /// @param  [in] filter Filter for SpaceCollideResult nearby objects to check
  /// @return ClosestResult Names, distances, and nearby points of nearby objects.
  /// @par Behavior:
  /// - Check if the object is created, enabled, extend_length is positive, top_n is 1 or more,
  ///   and the filter is non-zero. If invalid values are found, throw an exception.
  /// - Retrieve the object's AABB and create an extended box extend_object by extending it by extend_length.
  /// - Set the filter of extend_object to filter.
  /// - Disable the object.
  /// - Perform collision checks between extend_object and objects in the space. Sort the collision check results by distance between objects.
  /// - Enable the object.
  /// - Destroy extend_object.
  /// - Calculate the distance to up to top_n objects closest to the object.
  /// - Return the names, distances, and nearby points of the objects closest to the object in the space.
  /// @attention
  /// - If no objects collide with extend_object, ClosestResult.distance will be extend_length.
  /// - If the object is disabled, name will contain the object's name.
  virtual ClosestResult GetClosestObject(const std::string& name,
                                         double extend_length,
                                         int32_t top_n,
                                         uint16_t filter) = 0;

  /// @brief Retrieve the name of the physics engine in use
  /// @return std::string Name of the physics engine
  virtual std::string GetEngine() const = 0;

  /// @brief Ray casting function
  /// @param  [in] start_point Starting point of the ray
  /// @param  [in] direction Direction vector of the ray
  /// @param  [in] length Length of the ray
  /// @param  [out] end_point Coordinates on the object where the ray collided
  /// @param  [out] name Name of the object the ray collided with
  /// @return bool True if the ray collided with an object
  virtual bool RayCasting(const Eigen::Vector3d& start_point,
                          const Eigen::Vector3d& direction,
                          double length,
                          Eigen::Vector3d& end_point,
                          std::string& name) = 0;
};
}  // end namespace tmc_collision_detector
#endif  // TMC_COLLISION_DETECTOR_COLLISION_DETECTOR_HPP_
