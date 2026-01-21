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
/// @brief    Interference detection interface class
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
/// Interference check interface class
class ICollisionDetector {
 public:
  using Ptr = std::shared_ptr<ICollisionDetector>;

  virtual ~ICollisionDetector() {}

  /// @brief Create an object
  /// @param  [in,out] parameter Object information
  /// @par Behavior:
  /// - Check if parameter.shape.dimensions is non-negative. If negative, throw an exception InvalidShapeParamError.
  /// - Create a primitive according to parameter.shape.type.
  /// - In the case of a mesh, if the stl file cannot be read, throw an exception InvalidShapeParamError.
  /// - If type is not one of the values of CollisionObjectType, throw an exception NonExistTypeError.
  /// - Set the position and orientation according to parameter.transform.
  /// - Set the object's filter/group according to parameter.filter, parameter.group.
  virtual void CreateObject(
      const tmc_manipulation_types::ObjectParameter& parameter) = 0;

  /// @brief  Retrieve object information
  /// @param  [in] name Name of the object to retrieve
  /// @return ObjectParameter Object information
  /// @par Behavior:
  /// - Search for the object with the name in the list. If not found, throw an exception NonCreateError.
  /// - Return the object information.
  virtual tmc_manipulation_types::ObjectParameter GetObjectParameter(
      const std::string& name) const = 0;

  /// @brief  Retrieve the object's AABB
  /// @param  [in] name Name of the object to retrieve
  /// @return AABB Object's AABB [xmin xmax; ymin ymax; zmin zmax]
  /// @par Behavior:
  /// - Search for the object with the name in the list. If not found, throw an exception NonCreateError.
  /// - Retrieve the AABB.
  /// - Store and return the retrieved AABB.
  virtual tmc_manipulation_types::AABB GetObjectAABB(
      const std::string& name) const = 0;


  /// @brief  Destroy the object
  /// @param  [in] name Name of the object to destroy
  /// @par Behavior:
  /// - Search for the object with the name in the list. If not found, throw an exception NonCreateError.
  /// - Destroy the object.
  virtual void DestroyObject(const std::string& name) = 0;


  /// @brief  Destroy all objects after the anchor (excluding the anchor)
  /// @par Behavior:
  /// - Check if the anchor is set. If not set, throw an exception.
  /// - Sequentially destroy objects after the anchor.
  /// - If the anchor is the last object, do nothing.
  virtual void DestroyObject(void) = 0;

  /// @brief  Set the current last object as the anchor
  /// @par Behavior:
  /// - Check if objects exist in the environment. If not, throw an exception.
  /// - Set the last object as the anchor.
  virtual void SetAnchor(void) = 0;

  /// @brief  Set the position and orientation of the object
  /// @param  [in] transform Position and orientation to set
  /// @param  [in] name Name of the object to set
  /// @par Behavior:
  /// - Search for the object with the name in the list. If not found, throw an exception NonCreateError.
  /// - Update the position and orientation of the object.
  virtual void SetObjectTransform(const Eigen::Affine3d &transform,
                                  const std::string& name) = 0;

  /// @brief  Retrieve the position and orientation of the object
  /// @param  [in] name Name of the object to retrieve
  /// @return Eigen::Affine3d Retrieved position and orientation
  /// @par Behavior:
  /// - Search for the object with the name in the list. If not found, throw an exception NonCreateError.
  /// - Return the position and orientation of the object.
  virtual Eigen::Affine3d GetObjectTransform(
      const std::string& name) const = 0;

  /// @brief  Set the group of the object
  /// @param  [in] group Group bit to set
  /// @param  [in] name Name of the object to set
  /// @par Behavior:
  /// - Search for the object with the name in the list. If not found, throw an exception NonCreateError.
  /// - Set the group of the object.
  virtual void SetCollisionGroup(const uint16_t group,
                                 const std::string& name) = 0;

  /// @brief  Set the filter of the object
  /// @param  [in] filter Filter bit to set
  /// @param  [in] name Name of the object to set
  /// @par Behavior:
  /// - Search for the object with the name in the list. If not found, throw an exception NonCreateError.
  /// - Set the filter of the object.
  virtual void SetCollisionFilter(const uint16_t filter,
                                  const std::string& name) = 0;

  /// @brief  Retrieve the group of the object
  /// @param  [in] name Name of the object to retrieve
  /// @return uint16_t Group of the object
  /// @par Behavior:
  /// - Search for the object with the name in the list. If not found, throw an exception NonCreateError.
  /// - Retrieve the group of the object.
  virtual uint16_t GetCollisionGroup(const std::string& name) const = 0;

  /// @brief  Retrieve the filter of the object
  /// @param  [in] name Name of the object to retrieve
  /// @return uint16_t Filter of the object
  /// @par Behavior:
  /// - Search for the object with the name in the list. If not found, throw an exception NonCreateError.
  /// - Retrieve the filter of the object.
  virtual uint16_t GetCollisionFilter(const std::string& name) const = 0;

  /// @brief  Enable interference check for the object
  /// @param  [in] name Name of the object to enable interference check
  /// @par Behavior:
  /// - Search for the object with the name in the list. If not found, throw an exception NonCreateError.
  /// - Enable the object.
  virtual void EnableObject(const std::string& name) = 0;

  /// @brief  Disable interference check for the object
  /// @param  [in] name Name of the object to disable interference check
  /// @par Behavior:
  /// - Search for the object with the name in the list. If not found, throw an exception NonCreateError.
  /// - Disable the object.
  virtual void DisableObject(const std::string& name) = 0;

  /// @brief  Add object pairs to exclude from interference check in space
  /// @param  [in] names Pair of object names to exclude from interference check
  /// @par Behavior:
  /// - Search for the objects with names nameA, nameB in the list. If not found, throw an exception NonCreateError.
  /// - Search if the object pair is in the interference check addition pair list. If found, remove from the list.
  /// - If not found, add the object pair to the interference check exclusion pair list.
  /// @attention
  /// - Can set the state to not perform interference check separately from filter and interference check enable/disable.
  /// - By calling ResetCollisionCheckPairList() or specifying with EnableCollisionCheck()
  ///   can remove from the interference check exclusion pair list.
  /// - Can add object pairs already existing in the interference check exclusion pair list to the list again.
  virtual void DisableCollisionCheck(const std::vector<PairString>& names) = 0;

  /// @brief  Add object pairs to perform interference check in space
  /// @param  [in] names Pair of object names to add for interference check
  /// @par Behavior:
  /// - Search for the objects with names nameA, nameB in the list. If not found, throw an exception NonCreateError.
  /// - Search if the object pair is in the interference check exclusion pair list. If found, remove from the list.
  /// - If not found, add the object pair to the interference check addition pair list.
  /// @attention
  /// - Can set the state to perform interference check separately from filter and interference check enable/disable.
  /// - By calling ResetCollisionCheckPairList() or specifying with DisableCollisionCheck()
  ///   can remove from the interference check addition pair list.
  /// - Can add object pairs already existing in the interference check addition pair list to the list again.
  virtual void EnableCollisionCheck(const std::vector<PairString>& names) = 0;

  /// @brief  Clear the interference check exclusion pair list and addition pair list.
  /// @par Behavior:
  /// - Clear the interference check exclusion pair list and addition pair list.
  virtual void ResetCollisionCheckPairList() = 0;

  /// @brief  Check if two objects are interfering
  /// @param  [in] nameA Object to perform interference check
  /// @param  [in] nameB Object to perform interference check
  /// @return bool Result of the interference check between two objects. True if interfering
  /// @par Behavior:
  /// - Search for the objects with names nameA, nameB in the list. If not found, throw an exception NonCreateError.
  /// - Perform interference check between objects.
  /// - Return the result.
  /// @attention
  /// - Calculation is performed even if the object is disabled.
  virtual bool CheckCollisionPair(const std::string& nameA,
                                  const std::string& nameB) = 0;

  /// @brief  Check if two objects are interfering
  /// @param  [in] nameA Object to perform interference check
  /// @param  [in] nameB Object to perform interference check
  /// @param  [out] point Contact point on object nameB
  /// @param  [out] normal Normal from contact point on object nameB to object nameA
  /// @return bool Result of the interference check between two objects. True if interfering
  /// @par Behavior:
  /// - Search for the objects with names nameA, nameB in the list. If not found, throw an exception NonCreateError.
  /// - Perform interference check between objects.
  /// - Return the result.
  /// @attention
  /// - Calculation is performed even if the object is disabled.
  /// - If not interfering, point and normal will contain 0
  virtual bool CheckCollisionPair(const std::string& nameA,
                                  const std::string& nameB,
                                  Eigen::Vector3d& point,
                                  Eigen::Vector3d& normal) = 0;

  /// @brief  Check if two objects are interfering
  /// @param  [in] nameA Object to perform interference check
  /// @param  [in] nameB Object to perform interference check
  /// @param  [out] depth Depth if interfering
  /// @return bool Result of the interference check between two objects. True if interfering
  /// @attention
  /// - Calculation is performed even if the object is disabled.
  /// - If not interfering, depth will contain 0.0
  virtual bool CheckCollisionPair(const std::string& nameA,
                                  const std::string& nameB,
                                  double& depth) = 0;

  /// @brief  Check if objects in space are interfering
  /// @return bool Result of the interference check of objects. True if interfering
  /// @par Behavior:
  /// - Use the functions of each physics engine to perform interference check of objects in space
  /// - If interference occurs, terminate the interference check there.
  /// - Return the result.
  /// @attention
  /// - If there is one or fewer objects in space, the result of the interference check will be false.
  virtual bool CheckCollisionSpace(void) = 0;

  /// @brief  Check if objects in space are interfering and get the names of the interfering object pairs
  /// @param  [out] dst_contact_pair Pair of names of interfering objects
  /// @return bool Result of the interference check of objects. True if interfering
  /// @par Behavior:
  /// - Use the functions of each physics engine to perform interference check of objects in space.
  /// - If interference occurs, terminate the interference check there.
  /// - Get the names of the interfering objects. If names are not found, throw an exception InvalidObjectContactError.
  /// - Return the result.
  /// @attention
  /// - If there is one or fewer objects in space, the result of the interference check will be false.
  /// - If the result of the interference check is false, dst_contact_pair is not updated.
  virtual bool CheckCollisionSpace(PairString& dst_contact_pair) = 0;

  /// @brief  Create a list of pairs of interfering objects
  /// @param  [out] dst_contact_pair List of pairs of names of interfering objects
  /// @return bool Result of the interference check of objects. True if interfering
  /// @par Behavior:
  /// - Use the functions of each physics engine to perform interference check of objects in space
  /// - If interference occurs, get the names of the interfering objects.
  ///   If names are not found, throw an exception InvalidObjectContactError.
  /// - Return the result after the interference check of objects in space is completed.
  /// @attention
  /// - If there is one or fewer objects in space, the result of the interference check will be false.
  /// - If the result of the interference check is false, dst_contact_pair will be empty.
  virtual bool GetContactPairList(
      std::vector<PairString>& dst_contact_pair) = 0;

  /// @brief  Retrieve the distance between two objects
  /// @param  [in] nameA Object to perform interference check
  /// @param  [in] nameB Object to perform interference check
  /// @return ClosestResult Result of the interference check between two objects.
  /// @par Behavior:
  /// - Search for the objects with names nameA, nameB in the list. If not found, throw an exception NonCreateError.
  /// - Calculate the distance between objects.
  /// @attention
  /// - Calculation is performed even if the object is disabled.
  virtual ClosestResult GetClosestResult(const std::string& nameA,
                                         const std::string& nameB) = 0;

  /// @brief  Retrieve information of the nearest object in space for a given object
  /// @param  [in] name Name of the object for which to retrieve the nearest object
  /// @param  [in] extend_length Upper limit of the distance to the nearest object
  /// @param  [in] top_n Number of objects for which to calculate the exact distance using the GJK algorithm
  /// @param  [in] filter Filter for checking nearby objects in SpaceCollideResult
  /// @return ClosestResult Name, distance, and nearby point of the nearest object.
  /// @par Behavior:
  /// - Check if the object is created, enabled, extend_length is positive, top_n is 1 or more,
  ///   and filter is not 0. If invalid, throw an exception.
  /// - Retrieve the object's AABB and create an extended box extend_object by extending it by extend_length.
  /// - Set the filter of extend_object to filter.
  /// - Disable the object.
  /// - Perform interference check between extend_object and objects in space. Sort the results by distance between objects.
  /// - Enable the object.
  /// - Destroy extend_object.
  /// - Calculate the distance to the object for up to top_n closest objects.
  /// - Return the name, distance, and nearby point of the closest object in space as the result.
  /// @attention
  /// - If no objects interfere with extend_object, ClosestResult.distance will be extend_length.
  /// - If the object is disabled, the name of the object will be in name.
  virtual ClosestResult GetClosestObject(const std::string& name,
                                         double extend_length,
                                         int32_t top_n,
                                         uint16_t filter) = 0;

  /// @brief Retrieve the physics engine in use
  /// @return std::string Name of the physics engine
  virtual std::string GetEngine() const = 0;

  /// @brief  Ray casting function
  /// @param  [in] start_point Starting point of the ray
  /// @param  [in] direction Direction vector of the ray
  /// @param  [in] length Length of the ray
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
