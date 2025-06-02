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
/// @file     collision_detector.hpp
/// @brief    Interface class for interference detection
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

/// Structure to insert nearest objects, nearby points, and distance
struct ClosestResult {
  std::string name;  /// Name of the nearby object
  bool contact;  /// True if in contact
  double distance;  /// Distance between objects
  Eigen::Vector3d closest_point_on_A;  /// Nearby point on the object
  Eigen::Vector3d closest_point_on_B;  /// Nearby point on the nearby object
  Eigen::Vector3d normal_on_B;  // Normal on B
};
/// Interface class for interference checking
class ICollisionDetector {
 public:
  using Ptr = std::shared_ptr<ICollisionDetector>;

  virtual ~ICollisionDetector() {}

  /// @brief Object creation
  /// @param  [in,out] parameter Information of the object
  /// @par Behavior:
  /// - Check if parameter.shape.dimensions is non-negative. If negative, throw exception InvalidShapeParamError.
  /// - Create a primitive according to parameter.shape.type.
  /// - For meshes, if unable to read stl file, throw exception InvalidShapeParamError.
  /// - If type is not any value of CollisionObjectType, throw exception NonExistTypeError.
  /// - Set position orientation according to parameter.transform.
  /// - Set object's filter/group according to parameter.filter and parameter.group.
  virtual void CreateObject(
      const tmc_manipulation_types::ObjectParameter& parameter) = 0;

  /// @brief Retrieve information of the object
  /// @param  [in] name Name of the object to retrieve
  /// @return ObjectParameter Information of the object
  /// @par Behavior:
  /// - Search for the object with name name in the list. If not found, throw exception NonCreateError.
  /// - Return information of the object.
  virtual tmc_manipulation_types::ObjectParameter GetObjectParameter(
      const std::string& name) const = 0;

  /// @brief Retrieve AABB of the object
  /// @param  [in] name Name of the object to retrieve
  /// @return AABB AABB of the object [xmin xmax; ymin ymax; zmin zmax]
  /// @par Behavior:
  /// - Search for the object with name name in the list. If not found, throw exception NonCreateError.
  /// - Retrieve AABB.
  /// - Store the retrieved AABB and return.
  virtual tmc_manipulation_types::AABB GetObjectAABB(
      const std::string& name) const = 0;


  /// @brief Dispose of the object
  /// @param  [in] name Name of the object to dispose
  /// @par Behavior:
  /// - Search for the object with name name in the list. If not found, throw exception NonCreateError.
  /// - Dispose of the object.
  virtual void DestroyObject(const std::string& name) = 0;


  /// @brief Dispose all objects behind the anchor (excluding the anchor)
  /// @par Behavior:
  /// - Check if the anchor is set. If not set, throw exception.
  /// - Sequentially dispose of objects behind the anchor.
  /// - If the anchor is the last object, do nothing.
  virtual void DestroyObject(void) = 0;

  /// @brief Set the current last object as the anchor
  /// @par Behavior:
  /// - Confirm if there are objects in the environment. If none exist, throw exception.
  /// - Set the last object as the anchor.
  virtual void SetAnchor(void) = 0;

  /// @brief Set position orientation of the object
  /// @param  [in] transform Position orientation to set
  /// @param  [in] name Name of the object to set
  /// @par Behavior:
  /// - Search for the object with name name in the list. If not found, throw exception NonCreateError.
  /// - Update position orientation of the object.
  virtual void SetObjectTransform(const Eigen::Affine3d &transform,
                                  const std::string& name) = 0;

  /// @brief Retrieve position orientation of the object
  /// @param  [in] name Name of the object to retrieve
  /// @return Eigen::Affine3d Retrieved position orientation
  /// @par Behavior:
  /// - Search for the object with name name in the list. If not found, throw exception NonCreateError.
  /// - Return position orientation of the object.
  virtual Eigen::Affine3d GetObjectTransform(
      const std::string& name) const = 0;

  /// @brief Set group of the object
  /// @param  [in] group Group bit to set
  /// @param  [in] name Name of the object to set
  /// @par Behavior:
  /// - Search for the object with name name in the list. If not found, throw exception NonCreateError.
  /// - Set group of the object.
  virtual void SetCollisionGroup(const uint16_t group,
                                 const std::string& name) = 0;

  /// @brief Set filter of the object
  /// @param  [in] filter Filter bit to set
  /// @param  [in] name Name of the object to set
  /// @par Behavior:
  /// - Search for the object with name name in the list. If not found, throw exception NonCreateError.
  /// - Set filter of the object.
  virtual void SetCollisionFilter(const uint16_t filter,
                                  const std::string& name) = 0;

  /// @brief Retrieve group of the object
  /// @param  [in] name Name of the object to retrieve
  /// @return uint16_t Group of the object
  /// @par Behavior:
  /// - Search for the object with name name in the list. If not found, throw exception NonCreateError.
  /// - Retrieve group of the object.
  virtual uint16_t GetCollisionGroup(const std::string& name) const = 0;

  /// @brief Retrieve filter of the object
  /// @param  [in] name Name of the object to retrieve
  /// @return uint16_t Filter of the object
  /// @par Behavior:
  /// - Search for the object with name name in the list. If not found, throw exception NonCreateError.
  /// - Retrieve filter of the object.
  virtual uint16_t GetCollisionFilter(const std::string& name) const = 0;

  /// @brief Enable collision check for the object
  /// @param  [in] name Name of the object to enable collision check
  /// @par Behavior:
  /// - Search for the object with name name in the list. If not found, throw exception NonCreateError.
  /// - Enable the object.
  virtual void EnableObject(const std::string& name) = 0;

  /// @brief Disable collision check for the object
  /// @param  [in] name Name of the object to disable collision check
  /// @par Behavior:
  /// - Search for the object with name name in the list. If not found, throw exception NonCreateError.
  /// - Disable the object.
  virtual void DisableObject(const std::string& name) = 0;

  /// @brief Add object pair to exclude from collision check in space
  /// @param  [in] names Pair of object names to exclude from collision check
  /// @par Behavior:
  /// - Search for objects with names nameA,nameB in the list. If not found, throw exception NonCreateError.
  /// - Search if the object pair is in the collision check added pair list. If found, remove from collision check added pair list.
  /// - If not found, add the object pair to the collision check exclusion pair list.
  /// @attention
  /// - It can be set to a state that does not perform collision checks regardless of filter or collision check enable/disable.
  /// - By calling ResetCollisionCheckPairList() or specifying with EnableCollisionCheck()
  ///   you can remove from collision check exclusion pair list.
  /// - You can further add object pairs that already exist in the collision check exclusion pair list to the list.
  virtual void DisableCollisionCheck(const std::vector<PairString>& names) = 0;

  /// @brief Add object pair to perform collision check in space
  /// @param  [in] names Pair of object names to add and perform collision check
  /// @par Behavior:
  /// - Search for objects with names nameA,nameB in the list. If not found, throw exception NonCreateError.
  /// - Search if the object pair is in the collision check exclusion pair list. If found, remove from collision check exclusion pair list.
  /// - If not found, add the object pair to the collision check added pair list.
  /// @attention
  /// - It can be set to a state that performs collision checks regardless of filter or collision check enable/disable.
  /// - By calling ResetCollisionCheckPairList() or specifying with DisableCollisionCheck()
  ///   you can remove from collision check added pair list.
  /// - You can further add object pairs that already exist in the collision check added pair list to the list.
  virtual void EnableCollisionCheck(const std::vector<PairString>& names) = 0;

  /// @brief Clear collision check exclusion pair list and collision check added pair list.
  /// @par Behavior:
  /// - Clear collision check exclusion pair list and collision check added pair list.
  virtual void ResetCollisionCheckPairList() = 0;

  /// @brief Check if two objects are interfering
  /// @param  [in] nameA Object to perform collision check
  /// @param  [in] nameB Object to perform collision check
  /// @return bool Returns the result of the collision check between two objects. Returns true if interfering
  /// @par Behavior:
  /// - Search for objects with names nameA,nameB in the list. If not found, throw exception NonCreateError.
  /// - Perform collision check between objects.
  /// - Return the result.
  /// @attention
  /// - Calculation is performed even if object is Disabled.
  virtual bool CheckCollisionPair(const std::string& nameA,
                                  const std::string& nameB) = 0;

  /// @brief Check if two objects are interfering
  /// @param  [in] nameA Object to perform collision check
  /// @param  [in] nameB Object to perform collision check
  /// @param  [out] point Contact point on object nameB
  /// @param  [out] normal Normal from contact point on object nameB to object nameA
  /// @return bool Returns the result of the collision check between two objects. Returns true if interfering
  /// @par Behavior:
  /// - Search for objects with names nameA,nameB in the list. If not found, throw exception NonCreateError.
  /// - Perform collision check between objects.
  /// - Return the result.
  /// @attention
  /// - Calculation is performed even if object is Disabled.
  /// - If not interfering, 0 is inserted into point,normal
  virtual bool CheckCollisionPair(const std::string& nameA,
                                  const std::string& nameB,
                                  Eigen::Vector3d& point,
                                  Eigen::Vector3d& normal) = 0;


  /// @brief Check if objects in space are interfering
  /// @return bool Returns the result of the collision check of the objects. Returns true if interfering
  /// @par Behavior:
  /// - Utilize functions of each physics engine to perform collision check of objects in space
  /// - If interference occurs, stop the collision check there.
  /// - Return the result.
  /// @attention
  /// - If there are one or fewer objects in space, result of collision check will be false.
  virtual bool CheckCollisionSpace(void) = 0;

  /// @brief Check if objects in space are interfering, retrieve names of interfering object pairs
  /// @param  [out] dst_contact_pair Names of interfering object pairs
  /// @return bool Returns the result of the collision check of the objects. Returns true if interfering
  /// @par Behavior:
  /// - Utilize functions of each physics engine to perform collision check of objects in space.
  /// - If interference occurs, stop the collision check there.
  /// - Retrieve names of interfering objects. If names cannot be found, throw exception InvalidObjectContactError.
  /// - Return the result.
  /// @attention
  /// - If there are one or fewer objects in space, result of collision check will be false.
  /// - If result of collision check is false, dst_contact_pair will not be updated.
  virtual bool CheckCollisionSpace(PairString& dst_contact_pair) = 0;

  /// @brief Create a list of pairs of interfering objects
  /// @param  [out] dst_contact_pair List of pairs of names of interfering objects
  /// @return bool Returns the result of the collision check of the objects. Returns true if interfering
  /// @par Behavior:
  /// - Utilize functions of each physics engine to perform collision check of objects in space
  /// - If interference occurs, retrieve names of interfering objects.
  ///   If names cannot be found, throw exception InvalidObjectContactError.
  /// - Once collision check of objects in space is finished, return the result.
  /// @attention
  /// - If there are one or fewer objects in space, result of collision check will be false.
  /// - If result of collision check is false, dst_contact_pair will be empty.
  virtual bool GetContactPairList(
      std::vector<PairString>& dst_contact_pair) = 0;

  /// @brief Retrieve distance between two objects
  /// @param  [in] nameA Object to perform collision check
  /// @param  [in] nameB Object to perform collision check
  /// @return ClosestResult Returns the result of the collision check of two objects.
  /// @par Behavior:
  /// - Search for objects with names nameA,nameB in the list. If not found, throw exception NonCreateError.
  /// - Calculate distance between objects.
  /// @attention
  /// - Calculation is performed even if object is Disabled.
  virtual ClosestResult GetClosestResult(const std::string& nameA,
                                         const std::string& nameB) = 0;

  /// @brief For a certain object, retrieve information of the nearest object among objects in space
  /// @param  [in] name Name of the object to retrieve nearby object
  /// @param  [in] extend_length Upper limit of distance to nearby object
  /// @param  [in] top_n Number of objects for which the exact distance is calculated using the GJK algorithm
  /// @param  [in] filter Filter for SpaceCollideResult nearby object to check
  /// @return ClosestResult Names, distance, nearby points of nearby object.
  /// @par Behavior:
  /// - Check if object is created, enabled, extend_length is positive, top_n is 1 or higher,
  ///   filter is not 0, and throw exception if any value is invalid.
  /// - Retrieve AABB of the object and create a box extend_object that is expanded by extend_length.
  /// - Set filter to extend_object's filter.
  /// - Disable object.
  /// - Perform collision check between extend_object and objects in space. Sort collision check results by distance between objects.
  /// - Enable object.
  /// - Dispose of extend_object.
  /// - Determine distance for up to top_n closest objects from object.
  /// - Return name, distance, and nearby points of the closest object in space to object.
  /// @attention
  /// - If no object interferes with extend_object, ClosestResult.distance becomes extend_length.
  /// - If object is disabled, name contains object's name.
  virtual ClosestResult GetClosestObject(const std::string& name,
                                         double extend_length,
                                         int32_t top_n,
                                         uint16_t filter) = 0;

  /// @brief Retrieve the physics engine in use
  /// @return std::string Physics engine name
  virtual std::string GetEngine() const = 0;

  /// @brief Ray casting function
  /// @param  [in] start_point Start point of the ray
  /// @param  [in] direction Ray direction vector
  /// @param  [in] length Ray length
  /// @param  [out] end_point Coordinates on the object where the ray hit
  /// @param  [out] name Name of the object where the ray hit
  /// @return bool True if collided with an object
  virtual bool RayCasting(const Eigen::Vector3d& start_point,
                          const Eigen::Vector3d& direction,
                          double length,
                          Eigen::Vector3d& end_point,
                          std::string& name) = 0;
};
}  // end namespace tmc_collision_detector
#endif  // TMC_COLLISION_DETECTOR_COLLISION_DETECTOR_HPP_
