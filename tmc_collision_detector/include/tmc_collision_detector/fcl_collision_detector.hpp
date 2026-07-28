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
#ifndef TMC_COLLISION_DETECTOR_FCL_COLLISION_DETECTOR_HPP_
#define TMC_COLLISION_DETECTOR_FCL_COLLISION_DETECTOR_HPP_

#include <bitset>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/narrowphase/contact.h>

#include <tmc_collision_detector/collision_detector.hpp>

namespace tmc_collision_detector {

const char* const kFclName = "fcl";

class CollisionObject : public fcl::CollisionObjectf {
 public:
  explicit CollisionObject(const std::shared_ptr<fcl::CollisionGeometryf>& cgeom) : fcl::CollisionObjectf(cgeom) {}

  void computeAABBImproved() {
    if (t.linear().isIdentity()) {
      aabb = fcl::translate(cgeom->aabb_local, t.translation());
    } else {
      std::vector<Eigen::Vector3f> vertices = {
        {cgeom->aabb_local.min_.x(), cgeom->aabb_local.min_.y(), cgeom->aabb_local.min_.z()},
        {cgeom->aabb_local.min_.x(), cgeom->aabb_local.min_.y(), cgeom->aabb_local.max_.z()},
        {cgeom->aabb_local.min_.x(), cgeom->aabb_local.max_.y(), cgeom->aabb_local.min_.z()},
        {cgeom->aabb_local.min_.x(), cgeom->aabb_local.max_.y(), cgeom->aabb_local.max_.z()},
        {cgeom->aabb_local.max_.x(), cgeom->aabb_local.min_.y(), cgeom->aabb_local.min_.z()},
        {cgeom->aabb_local.max_.x(), cgeom->aabb_local.min_.y(), cgeom->aabb_local.max_.z()},
        {cgeom->aabb_local.max_.x(), cgeom->aabb_local.max_.y(), cgeom->aabb_local.min_.z()},
        {cgeom->aabb_local.max_.x(), cgeom->aabb_local.max_.y(), cgeom->aabb_local.max_.z()}};

      for (auto& vertex : vertices) {
        vertex = t.linear() * vertex;
      }

      aabb.min_ = vertices[0];
      aabb.max_ = vertices[0];
      for (const auto& vertex : vertices) {
        aabb.min_ = aabb.min_.cwiseMin(vertex);
        aabb.max_ = aabb.max_.cwiseMax(vertex);
      }
      aabb = fcl::translate(aabb, t.translation());
    }
  }
};


class FclCollisionDetector : public ICollisionDetector {
 public:
  FclCollisionDetector();
  virtual ~FclCollisionDetector();

  /// Create an object
  void CreateObject(const tmc_manipulation_types::ObjectParameter& parameter) override;
  /// Retrieve object parameters
  tmc_manipulation_types::ObjectParameter GetObjectParameter(const std::string& name) const override;
  /// Retrieve the object's AABB
  tmc_manipulation_types::AABB GetObjectAABB(const std::string& name) const override;

  /// Destroy the object
  void DestroyObject(const std::string& name) override;
  /// Destroy all objects behind the anchor (excluding the anchor)
  void DestroyObject(void) override;
  /// Set the current last object as the anchor
  void SetAnchor(void) override;

  /// Set the position and orientation of the object
  void SetObjectTransform(const Eigen::Affine3d& transform, const std::string& name) override;
  /// Get the position and orientation of the object
  Eigen::Affine3d GetObjectTransform(const std::string& name) const override;

  /// Set the object's group
  void SetCollisionGroup(const uint16_t group, const std::string& name) override;
  /// Set the object's filter
  void SetCollisionFilter(const uint16_t filter, const std::string& name) override;
  /// Get the object's group
  uint16_t GetCollisionGroup(const std::string& name) const override;
  /// Get the object's filter
  uint16_t GetCollisionFilter(const std::string& name) const override;

  /// Enable interference checking for the object
  void EnableObject(const std::string& name) override;
  /// Disable interference checking for the object
  void DisableObject(const std::string& name) override;

  /// Add object pairs to exclude from interference checking in the space
  void DisableCollisionCheck(const std::vector<PairString>& name_pairs) override;
  /// Add object pairs to perform interference checking in the space
  void EnableCollisionCheck(const std::vector<PairString>& name_pairs) override;

  /// Discard the exclusion list
  void ResetCollisionCheckPairList() override;

  /// Check if two objects are interfering
  bool CheckCollisionPair(const std::string& nameA, const std::string& nameB) override;

  /// Check if two objects are interfering (return contact information)
  bool CheckCollisionPair(const std::string& nameA, const std::string& nameB,
                          Eigen::Vector3d& point, Eigen::Vector3d& normal) override;

  /// Check if two objects are interfering (return contact depth)
  bool CheckCollisionPair(const std::string& nameA, const std::string& nameB, double& depth) override;

  /// Check if objects in the space are interfering
  bool CheckCollisionSpace(void) override;
  /// Interference check to get the names of interfering object pairs
  bool CheckCollisionSpace(PairString& dst_contact_pair) override;
  /// Create a list of interfering object pairs
  bool GetContactPairList(std::vector<PairString>& dst_contact_pair) override;
  /// Get the distance between two objects
  ClosestResult GetClosestResult(const std::string& nameA, const std::string& nameB) override {
    throw std::runtime_error("Not implemented: GetClosestResult");
  }
  /// Retrieve information about the nearest object
  ClosestResult GetClosestObject(const std::string& name,
                                 double extend_length,
                                 int32_t top_n,
                                 uint16_t filter) override {
    throw std::runtime_error("Not implemented: GetClosestObject");
  }

  /// Get the physics engine in use
  std::string GetEngine() const override {return kFclName;}

  /// Ray casting function
  bool RayCasting(const Eigen::Vector3d& start_point,
                  const Eigen::Vector3d& direction,
                  double length,
                  Eigen::Vector3d& end_point,
                  std::string& name) override {
    throw std::runtime_error("Not implemented: RayCasting");
  }

 private:
  // Object information
  struct ObjectInfo {
    std::string name;
    uint32_t group_index;
    bool is_pose_changed;
    bool is_protected;
    bool is_mesh;

    explicit ObjectInfo(const std::string& _name)
        : name(_name), group_index(0), is_pose_changed(true), is_protected(false), is_mesh(false) {}
    ObjectInfo(const std::string& _name, uint32_t _group_index, const tmc_manipulation_types::Shape& shape)
        : name(_name), group_index(_group_index), is_pose_changed(true), is_protected(false),
          is_mesh(shape.type == tmc_manipulation_types::kMesh ||
                  shape.type == tmc_manipulation_types::kMeshVertices) {}

    bool operator==(const ObjectInfo& rhs) const {
      return name == rhs.name;
    }
  };
  struct CollisionObjectWithInfo {
    std::shared_ptr<CollisionObject> collision_object;
    std::shared_ptr<ObjectInfo> object_info;

    CollisionObjectWithInfo(const std::shared_ptr<CollisionObject>& _collision_object,
                            const std::shared_ptr<ObjectInfo>& _object_info)
        : collision_object(_collision_object), object_info(_object_info) {
      // Although I want to avoid using const_cast, it's done this way because adding it during geometry generation would be redundant and cumbersome
      const_cast<fcl::CollisionGeometryf*>(collision_object->collisionGeometry().get())
          ->setUserData(object_info.get());
    }
  };

  bool CheckCollisionSpaceImpl(bool end_flag, std::vector<PairString>& dst_contact_pair);
  static bool CollisionFunction(fcl::CollisionObjectf* o1, fcl::CollisionObjectf* o2, void* data);

  std::vector<std::shared_ptr<fcl::BroadPhaseCollisionManagerf>> managers_;
  std::vector<bool> do_manager_update_;
  std::vector<bool> is_protected_;
  // This "16" is a remnant of ODE's specifications, which is not ideal
  // Forcing the filter to be the same if the group is the same doesn't align with the tmc_collision_detector interface
  std::vector<std::bitset<16>> filters_;

  std::map<std::string, CollisionObjectWithInfo> collision_objects_;
  bool anchor_called_;

  std::vector<PairString> exclusion_list_;
  std::vector<PairString> additional_list_;

  void ComputeAABB(CollisionObjectWithInfo& collision_object);

  std::optional<fcl::Contactf> CheckCollisionPairImpl(const std::string& nameA, const std::string& nameB);
};

}  // namespace tmc_collision_detector
#endif  // TMC_COLLISION_DETECTOR_FCL_COLLISION_DETECTOR_HPP_
