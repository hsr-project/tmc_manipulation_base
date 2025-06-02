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
/// @file     ODE_collision_detector.hpp
/// @brief    Interference detection library using ODE
/// @author   Keisuke Takeshita
/// @version  1.0.0
/// @date     2012.05.24
#ifndef TMC_COLLISION_DETECTOR_ODE_COLLISION_DETECTOR_HPP_
#define TMC_COLLISION_DETECTOR_ODE_COLLISION_DETECTOR_HPP_

#ifndef dDOUBLE
#define dDOUBLE
#endif

#include  <list>
#include  <map>
#include  <string>
#include  <utility>
#include  <vector>

#include  <ode/ode.h>

#include  <tmc_collision_detector/collision_detector.hpp>

namespace tmc_collision_detector {

/// Reserve value for the vector storing interference objects, for speed optimization
const int32_t kReserveContactPairNum = 1000;

/// Name of ODE
const char* const kODEName = "ODE";

using dGeomPair = std::pair<dGeomID, dGeomID>;
using ExclusionPair = std::pair<PairString, dGeomPair>;

/// Used when terminating the check upon interference in CheckCollisionSpace
struct SpaceCollideResult {
  dContact contact;
  bool result;
  dGeomPair id;
  std::vector<dGeomPair> exclusion_list;
};
/// Used when creating a list of interfering objects in CheckCollisionSpace
struct ContactPairResult {
  dContact contact;
  std::vector<dGeomPair> contact_list;
};
/// For RayCasting
struct RayCastingResult {
  dContactGeom contact;
  Eigen::Vector3d end_point;
  double distance;
  dGeomID object_id;
};
/// CollisionObject for ODE
struct ODECollisionObject {
  std::string name;                    /// Object name
  tmc_manipulation_types::Shape shape;     /// Object shape
  dGeomID object_id;                   /// Object ID
  dTriMeshDataID mesh_id;              /// Mesh ID
  std::vector<double> vertices;       /// Mesh vertex coordinates
  std::vector<uint32_t> indices;       /// Mesh vertex indices
};

/// Interference check class using ODE
class ODECollisionDetector : public ICollisionDetector {
 public:
  ODECollisionDetector();
  virtual ~ODECollisionDetector();

  /// Object creation
  virtual void CreateObject(
      const tmc_manipulation_types::ObjectParameter& parameter);
  /// Get object parameters
  virtual tmc_manipulation_types::ObjectParameter GetObjectParameter(
      const std::string& name) const;
  /// Get object AABB
  virtual tmc_manipulation_types::AABB GetObjectAABB(
      const std::string& name) const;

  /// Object disposal
  virtual void DestroyObject(const std::string& name);
  /// Dispose of all objects behind the anchor (excluding the anchor)
  virtual void DestroyObject(void);

  /// Make the current last object the anchor
  virtual void SetAnchor(void);

  /// Set the position and orientation of the object
  virtual void SetObjectTransform(const Eigen::Affine3d &transform,
                                  const std::string& name);
  /// Get the position and orientation of the object
  virtual Eigen::Affine3d GetObjectTransform(const std::string& name) const;

  /// Set the group of the object
  virtual void SetCollisionGroup(const uint16_t group,
                                 const std::string& name);
  /// Set the filter of the object
  virtual void SetCollisionFilter(const uint16_t filter,
                                  const std::string& name);
  /// Get the group of the object
  virtual uint16_t GetCollisionGroup(const std::string& name) const;
  /// Get the filter of the object
  virtual uint16_t GetCollisionFilter(const std::string& name) const;

  /// Enable interference check for the object
  virtual void EnableObject(const std::string& name);
  /// Disable interference check for the object
  virtual void DisableObject(const std::string& name);

  /// Add pairs of objects to exclude from interference check in the space
  virtual void DisableCollisionCheck(const std::vector<PairString>& names);
  /// Add pairs of objects to be checked for interference in the space
  virtual void EnableCollisionCheck(const std::vector<PairString>& names);

  /// Discard the exclusion list
  virtual void ResetCollisionCheckPairList();

  /// Check if two objects are interfering
  virtual bool CheckCollisionPair(const std::string& nameA,
                                  const std::string& nameB);
  /// Check if two objects are interfering (returns contact information)
  virtual bool CheckCollisionPair(const std::string& nameA,
                                  const std::string& nameB,
                                  Eigen::Vector3d& point,
                                  Eigen::Vector3d& normal);

  /// Check if objects in the space are interfering
  virtual bool CheckCollisionSpace(void);
  /// Interference check to obtain the names of pairs of interfering objects
  virtual bool CheckCollisionSpace(PairString& dst_contact_pair);
  /// Create a list of pairs of interfering objects
  virtual bool GetContactPairList(std::vector<PairString>& dst_contact_pair);
  /// Get the distance between two objects
  virtual ClosestResult GetClosestResult(const std::string& nameA,
                                         const std::string& nameB);
  /// Get the information of the nearest object
  virtual ClosestResult GetClosestObject(const std::string& name,
                                         double extend_length,
                                         int32_t top_n,
                                         uint16_t filter);

  /// Get the physics engine in use
  virtual std::string GetEngine() const {return std::string(kODEName);}

  /// Ray casting function
  virtual bool RayCasting(const Eigen::Vector3d& start_point,
                          const Eigen::Vector3d& direction,
                          double length,
                          Eigen::Vector3d& end_point,
                          std::string& name);

 private:
  /// ODE space
  dSpaceID space_;

  /// Object list
  std::list<ODECollisionObject> object_list_;

  /// Anchor
  std::list<ODECollisionObject>::iterator object_anchor_;
  /// Flag indicating whether the anchor is called
  bool anchor_called_;

  /// Get ODECollisionObject from object name
  std::list<ODECollisionObject>::const_iterator GetODECollisionObjectConst_(
      const std::string& name) const;
  std::list<ODECollisionObject>::iterator GetODECollisionObject_(
      const std::string& name);

  /// Used when terminating the check upon interference in CheckCollisionSpace
  static void SpaceCollideCallback_(void* data, dGeomID o1, dGeomID o2);

  /// Used when creating a list of interfering objects in CheckCollisionSpace
  static void MakeContactPairListCallback_(void* data, dGeomID o1, dGeomID o2);

  /// For RayCasting
  static void RayCastingCallback_(void* data, dGeomID o1, dGeomID o2);

  /// Get object name
  std::string GetObjectName_(dGeomID id) const;

  /// List of pairs of objects to remove from interference check
  std::vector<ExclusionPair> exclusion_list_;

  /// List of pairs of objects to add to interference check
  std::vector<PairString> add_pair_list_;

  /// Map of names and entities
  std::map<std::string, ODECollisionObject*> name_map_;
};
}  // end namespace tmc_collision_detector
#endif  // TMC_COLLISION_DETECTOR_ODE_COLLISION_DETECTOR_HPP_

