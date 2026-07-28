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

#include <tmc_collision_detector/fcl_collision_detector.hpp>

#include <chrono>

#include <fcl/broadphase/broadphase_bruteforce.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree_array.h>
#include <fcl/broadphase/broadphase_interval_tree.h>
#include <fcl/broadphase/broadphase_SaP.h>
#include <fcl/broadphase/broadphase_spatialhash.h>
#include <fcl/broadphase/broadphase_SSaP.h>

#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/narrowphase/collision.h>

#include <tmc_stl_loader/stl_loader.hpp>

#include "utils.hpp"

namespace {
constexpr uint32_t kMaxGroup = 16;

uint32_t ToGroupIndex(const uint16_t _group) {
  std::bitset<kMaxGroup> group(_group);
  if (group.count() == 0) {
    throw std::domain_error("ToGroupIndex error: group is empty");
  } else if (group.count() > 1) {
    throw std::domain_error("ToGroupIndex error: group is not unique");
  }
  for (auto i = 0u; i < kMaxGroup; ++i) {
    if (group.test(i)) {
      return i;
    }
  }
  throw std::domain_error("ToGroupIndex error: group is invalid");
}

std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> ToMeshModel(
    const std::vector<Eigen::Vector3f>& vertices, const std::vector<uint32_t>& indices) {
  std::vector<fcl::Triangle> triangles;
  for (int i = 0; i < indices.size() / 3; ++i) {
    triangles.push_back(fcl::Triangle(indices[3 * i], indices[3 * i + 1], indices[3 * i + 2]));
  }
  auto model = std::make_shared<fcl::BVHModel<fcl::OBBRSS<float>>>();
  model->beginModel();
  model->addSubModel(vertices, triangles);
  model->endModel();
  return model;
}

}  // namespace

namespace tmc_collision_detector {

std::shared_ptr<CollisionObject> ToCollisionObject(const tmc_manipulation_types::Shape& shape) {
  switch (shape.type) {
    case tmc_manipulation_types::kMesh: {
      auto stl_loader = std::make_shared<tmc_stl_loader::STLLoader>();
      tmc_stl_loader::Mesh mesh;
      stl_loader->Load(shape.filename, mesh);
      if (mesh.vertices.empty() || mesh.indices.empty()) {
        throw std::domain_error("ToCollisionObject error: invalid mesh file.");
      }
      auto model = ToMeshModel(mesh.vertices, mesh.indices);
      return std::make_shared<CollisionObject>(model);
    }
    case tmc_manipulation_types::kMeshVertices: {
      if (shape.vertices.empty() || shape.indices.empty()) {
        throw std::domain_error("ToCollisionObject error: invalid mesh vertices.");
      }
      auto model = ToMeshModel(shape.vertices, shape.indices);
      return std::make_shared<CollisionObject>(model);
    }
    case tmc_manipulation_types::kSphere: {
      if (shape.dimensions.size() != 1 || shape.dimensions[0] <= 0.0) {
        throw std::domain_error("ToCollisionObject error: invalid sphere shape");
      }
      auto model = std::make_shared<fcl::Sphere<float>>(shape.dimensions[0]);
      return std::make_shared<CollisionObject>(model);
    }
    case tmc_manipulation_types::kBox: {
      if (shape.dimensions.size() != 3 ||
          shape.dimensions[0] <= 0.0 || shape.dimensions[1] <= 0.0 || shape.dimensions[2] <= 0.0) {
        throw std::domain_error("ToCollisionObject error: invalid box shape");
      }
      auto model = std::make_shared<fcl::Box<float>>(shape.dimensions[0], shape.dimensions[1], shape.dimensions[2]);
      return std::make_shared<CollisionObject>(model);
    }
    case tmc_manipulation_types::kCylinder: {
      if (shape.dimensions.size() != 2 || shape.dimensions[0] <= 0.0 || shape.dimensions[1] <= 0.0) {
        throw std::domain_error("ToCollisionObject error: invalid cylinder shape");
      }
      auto model = std::make_shared<fcl::Cylinder<float>>(shape.dimensions[0], shape.dimensions[1]);
      return std::make_shared<CollisionObject>(model);
    }
    case tmc_manipulation_types::kCapsule: {
      if (shape.dimensions.size() != 2 || shape.dimensions[0] <= 0.0 || shape.dimensions[1] <= 0.0) {
        throw std::domain_error("ToCollisionObject error: invalid capsule shape");
      }
      auto model = std::make_shared<fcl::Capsule<float>>(shape.dimensions[0], shape.dimensions[1]);
      return std::make_shared<CollisionObject>(model);
    }
    default:
      throw std::domain_error("error: not supported primitive type");
  }
}

FclCollisionDetector::FclCollisionDetector() : anchor_called_(false) {
  for (auto i = 0u; i < kMaxGroup; ++i) {
    managers_.push_back(std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>());
    do_manager_update_.push_back(true);
    is_protected_.push_back(false);
  }
  filters_.resize(kMaxGroup);
}

FclCollisionDetector::~FclCollisionDetector() {
}

// Create an object
void FclCollisionDetector::CreateObject(const tmc_manipulation_types::ObjectParameter& parameter) {
  const auto group_index = ToGroupIndex(parameter.group);
  auto object_with_info = CollisionObjectWithInfo(
      ToCollisionObject(parameter.shape),
      std::make_shared<ObjectInfo>(parameter.name, group_index, parameter.shape));
  if (!anchor_called_) {
    is_protected_[group_index] = true;
    object_with_info.object_info->is_protected = true;
  }

  // TODO(Takeshita) ToCollisionObjectの処理にまとめる
  object_with_info.collision_object->setTranslation(parameter.transform.translation().cast<float>());
  object_with_info.collision_object->setRotation(parameter.transform.linear().cast<float>());
  ComputeAABB(object_with_info);

  managers_[group_index]->registerObject(object_with_info.collision_object.get());
  do_manager_update_[group_index] = true;
  filters_[group_index] = std::bitset<kMaxGroup>(parameter.filter);

  collision_objects_.insert(std::make_pair(parameter.name, object_with_info));
}

// Retrieve object parameters
tmc_manipulation_types::ObjectParameter FclCollisionDetector::GetObjectParameter(const std::string& name) const {
  auto it = collision_objects_.find(name);
  if (it == collision_objects_.end()) {
    throw std::domain_error("GetObjectParameter error: not exist object name: " + name);
  }
  tmc_manipulation_types::ObjectParameter parameter;
  parameter.name = name;
  parameter.transform = Eigen::Affine3f(it->second.collision_object->getTransform()).cast<double>();

  // The following is not implemented
  // parameter.group = ;
  // parameter.filter = ;
  // parameter.shape = ;
  return parameter;
}

// Retrieve the object's AABB
tmc_manipulation_types::AABB FclCollisionDetector::GetObjectAABB(const std::string& name) const {
  auto it = collision_objects_.find(name);
  if (it == collision_objects_.end()) {
    throw std::domain_error("GetObjectAABB error: not exist object name: " + name);
  }
  // Would like to use computeAABB, but it's non-const so it can't be used
  it->second.collision_object->computeAABBImproved();
  const auto aabb = it->second.collision_object->getAABB();
  tmc_manipulation_types::AABB result;
  result << aabb.min_[0], aabb.max_[0], aabb.min_[1], aabb.max_[1], aabb.min_[2], aabb.max_[2];
  return result;
}

// Destroy the object
void FclCollisionDetector::DestroyObject(const std::string& name) {
  auto it = collision_objects_.find(name);
  if (it == collision_objects_.end()) {
    throw std::domain_error("DestroyObject error: not exist object name: " + name);
  }

  managers_[it->second.object_info->group_index]->unregisterObject(it->second.collision_object.get());
  do_manager_update_[it->second.object_info->group_index] = true;

  collision_objects_.erase(it);

  for (auto it = additional_list_.begin(); it != additional_list_.end(); ) {
    if ((it->first == name) || (it->second == name)) {
      it = additional_list_.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = exclusion_list_.begin(); it != exclusion_list_.end(); ) {
    if ((it->first == name) || (it->second == name)) {
      it = exclusion_list_.erase(it);
    } else {
      ++it;
    }
  }
}

// Destroy all objects after the anchor (excluding the anchor)
void FclCollisionDetector::DestroyObject() {
  if (anchor_called_ == false) {
    throw std::domain_error("DestroyObject error: not set anchor");
  }
  // Non-protected manager initialization
  for (auto i = 0u; i < kMaxGroup; ++i) {
    if (!is_protected_[i]) {
      managers_[i]->clear();
      do_manager_update_[i] = true;
    }
  }

  // Flag management is overly complex, but register/unregister in the manager is a heavy process, so it can't be helped
  // First, the basic rule is to keep objects if they are protected, otherwise delete them
  // - If the retained object belongs to a non-protected manager, it was deleted above, so register it
  // - If the deleted object belongs to a protected manager, it needs to be removed, so unregister it
  for (auto it = collision_objects_.begin(); it != collision_objects_.end(); ) {
    if (it->second.object_info->is_protected) {
      if (!is_protected_[it->second.object_info->group_index]) {
        managers_[it->second.object_info->group_index]->registerObject(it->second.collision_object.get());
        do_manager_update_[it->second.object_info->group_index] = true;
      }
      ++it;
    } else {
      if (is_protected_[it->second.object_info->group_index]) {
        managers_[it->second.object_info->group_index]->unregisterObject(it->second.collision_object.get());
        do_manager_update_[it->second.object_info->group_index] = true;
      }
      it = collision_objects_.erase(it);
    }
  }
  // Strictly speaking, resetting might not be ideal, but it follows the conventional behavior of ODE
  ResetCollisionCheckPairList();
}

// Set the current last object as the anchor
void FclCollisionDetector::SetAnchor() {
  if (collision_objects_.empty()) {
    throw std::domain_error("SetAnchor error: space don't have object");
  }
  anchor_called_ = true;
}

// Set the position and orientation of the object
void FclCollisionDetector::SetObjectTransform(const Eigen::Affine3d& transform, const std::string& name) {
  auto it = collision_objects_.find(name);
  if (it == collision_objects_.end()) {
    throw std::domain_error("SetObjectTransform error: not exist object name: " + name);
  }
  it->second.collision_object->setTranslation(transform.translation().cast<float>());
  it->second.collision_object->setRotation(transform.linear().cast<float>());
  it->second.object_info->is_pose_changed = true;
}

// Retrieve the position and orientation of the object
Eigen::Affine3d FclCollisionDetector::GetObjectTransform(const std::string& name) const {
  auto it = collision_objects_.find(name);
  if (it == collision_objects_.end()) {
    throw std::domain_error("GetObjectTransform error: not exist object name: " + name);
  }
  return Eigen::Affine3f(it->second.collision_object->getTransform()).cast<double>();
}

// Set the object's group
void FclCollisionDetector::SetCollisionGroup(const uint16_t group, const std::string& name) {
  auto it = collision_objects_.find(name);
  if (it == collision_objects_.end()) {
    throw std::domain_error("SetCollisionGroup error: not exist object name: " + name);
  }
  // Since group exception checks are also performed, retrieve the group_index first
  const auto group_index = ToGroupIndex(group);

  managers_[it->second.object_info->group_index]->unregisterObject(it->second.collision_object.get());
  do_manager_update_[it->second.object_info->group_index] = true;

  managers_[group_index]->registerObject(it->second.collision_object.get());
  do_manager_update_[group_index] = true;
  it->second.object_info->group_index = group_index;
}

// Set the object's filter
void FclCollisionDetector::SetCollisionFilter(const uint16_t filter, const std::string& name) {
  auto it = collision_objects_.find(name);
  if (it == collision_objects_.end()) {
    throw std::domain_error("SetCollisionFilter error: not exist object name: " + name);
  }
  filters_[it->second.object_info->group_index] = std::bitset<kMaxGroup>(filter);
}

// Retrieve the object's group
uint16_t FclCollisionDetector::GetCollisionGroup(const std::string& name) const {
  auto it = collision_objects_.find(name);
  if (it == collision_objects_.end()) {
    throw std::domain_error("GetCollisionGroup error: not exist object name: " + name);
  }
  return 1 << it->second.object_info->group_index;
}

// Retrieve the object's filter
uint16_t FclCollisionDetector::GetCollisionFilter(const std::string& name) const {
  auto it = collision_objects_.find(name);
  if (it == collision_objects_.end()) {
    throw std::domain_error("GetCollisionFilter error: not exist object name: " + name);
  }
  return filters_[it->second.object_info->group_index].to_ulong();
}

// Enable collision checking for the object
void FclCollisionDetector::EnableObject(const std::string& name) {
  auto it = collision_objects_.find(name);
  if (it == collision_objects_.end()) {
    throw std::domain_error("EnableObject error: not exist object name: " + name);
  }
  managers_[it->second.object_info->group_index]->registerObject(it->second.collision_object.get());
  do_manager_update_[it->second.object_info->group_index] = true;
}

// Disable collision checking for the object
void FclCollisionDetector::DisableObject(const std::string& name) {
  auto it = collision_objects_.find(name);
  if (it == collision_objects_.end()) {
    throw std::domain_error("EnableObject error: not exist object name: " + name);
  }
  managers_[it->second.object_info->group_index]->unregisterObject(it->second.collision_object.get());
  do_manager_update_[it->second.object_info->group_index] = true;
}

// Compare two std::pair<std::string, std::string>
class IsEqualPairString {
 public:
  explicit IsEqualPairString(const PairString& names) : names_(names) {}
  bool operator()(const PairString& pair) const {
    if (((pair.first == names_.first) && (pair.second == names_.second)) ||
        ((pair.first == names_.second) && (pair.second == names_.first))) {
      return true;
    } else {
      return false;
    }
  }

 private:
  PairString names_;
};

// Add object pairs to exclude from collision checks in the space
void FclCollisionDetector::DisableCollisionCheck(const std::vector<PairString>& name_pairs) {
  for (const auto& name_pair : name_pairs) {
    auto it = std::find_if(additional_list_.begin(), additional_list_.end(), IsEqualPairString(name_pair));
    if (it == additional_list_.end()) {
      if (std::find_if(exclusion_list_.begin(),
                       exclusion_list_.end(),
                       IsEqualPairString(name_pair)) == exclusion_list_.end()) {
        exclusion_list_.push_back(name_pair);
      }
    } else {
      additional_list_.erase(it);
    }
  }
}

// Add object pairs to perform collision checks in the space
void FclCollisionDetector::EnableCollisionCheck(const std::vector<PairString>& name_pairs) {
  for (const auto& name_pair : name_pairs) {
    auto it = std::find_if(exclusion_list_.begin(), exclusion_list_.end(), IsEqualPairString(name_pair));
    if (it == exclusion_list_.end()) {
      if (std::find_if(additional_list_.begin(),
                       additional_list_.end(),
                       IsEqualPairString(name_pair)) == additional_list_.end()) {
        additional_list_.push_back(name_pair);
      }
    } else {
      exclusion_list_.erase(it);
    }
  }
}

// Discard the exclusion list
void FclCollisionDetector::ResetCollisionCheckPairList() {
  exclusion_list_.clear();
  additional_list_.clear();
}

/// Check if two objects are colliding
bool FclCollisionDetector::CheckCollisionPair(const std::string& nameA, const std::string& nameB) {
  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  return CheckCollisionPair(nameA, nameB, point, normal);
}

// Check if two objects are colliding (returns contact information)
bool FclCollisionDetector::CheckCollisionPair(const std::string& nameA, const std::string& nameB,
                                              Eigen::Vector3d& point, Eigen::Vector3d& normal) {
  const auto contact = CheckCollisionPairImpl(nameA, nameB);
  if (contact) {
    point = contact->pos.cast<double>();
    normal = contact->normal.cast<double>();
    return true;
  } else {
    point.setZero();
    normal.setZero();
    return false;
  }
}

// Check if two objects are colliding (returns contact depth)
bool FclCollisionDetector::CheckCollisionPair(const std::string& nameA, const std::string& nameB, double& depth) {
  const auto contact = CheckCollisionPairImpl(nameA, nameB);
  if (contact) {
    // Since it passes through CheckCollisionPairImpl, it is guaranteed that itA and itB exist
    auto itA = collision_objects_.find(nameA);
    auto itB = collision_objects_.find(nameB);
    if (itA->second.object_info->is_mesh && itB->second.object_info->is_mesh) {
      // In FCL 0.7, depth between meshes cannot be obtained, so calculate it with AABB as a placeholder
      // This way, changes in depth can be obtained, though not accurately
      // TODO(Takeshita) もう少し正しい計算を実装する
      const auto aabbA = itA->second.collision_object->getAABB();
      const auto aabbB = itB->second.collision_object->getAABB();
      depth = std::min(
          {std::max(0.0f, std::min(aabbA.max_[0], aabbB.max_[0]) - std::max(aabbA.min_[0], aabbB.min_[0])),
           std::max(0.0f, std::min(aabbA.max_[1], aabbB.max_[1]) - std::max(aabbA.min_[1], aabbB.min_[1])),
           std::max(0.0f, std::min(aabbA.max_[2], aabbB.max_[2]) - std::max(aabbA.min_[2], aabbB.min_[2]))});
    } else {
      // In combinations of meshes and primitives, negative values can occur; specifically, I observed negatives with meshes and spheres
      depth = std::abs(contact->penetration_depth);
    }
    return true;
  } else {
    depth = 0.0;
    return false;
  }
}

// Check if objects in the space are colliding
bool FclCollisionDetector::CheckCollisionSpace(void) {
  std::vector<PairString> contact_pair;
  return CheckCollisionSpaceImpl(true, contact_pair);
}

// Collision check to retrieve the names of colliding object pairs
bool FclCollisionDetector::CheckCollisionSpace(PairString& dst_contact_pair) {
  std::vector<PairString> contact_pair;
  const auto result = CheckCollisionSpaceImpl(true, contact_pair);
  if (result) {
    dst_contact_pair = contact_pair.front();
  }
  return result;
}

// Create a list of colliding object pairs
bool FclCollisionDetector::GetContactPairList(std::vector<PairString>& dst_contact_pair) {
  return CheckCollisionSpaceImpl(false, dst_contact_pair);
}

struct CollisionData {
  fcl::CollisionRequest<float> request;
  fcl::CollisionResult<float> result;

  bool done{false};

  std::vector<PairString>& exclusion_list_;

  explicit CollisionData(std::vector<PairString>& exclusion_list) : exclusion_list_(exclusion_list) {}
};

bool FclCollisionDetector::CheckCollisionSpaceImpl(bool end_flag, std::vector<PairString>& dst_contact_pair) {
  for (auto object : collision_objects_) {
    ComputeAABB(object.second);
  }
  for (auto i = 0u; i < kMaxGroup; ++i) {
    if (do_manager_update_[i]) {
      managers_[i]->update();
      do_manager_update_[i] = false;
    }
  }

  for (auto i = 0u; i < kMaxGroup; ++i) {
    if (managers_[i]->empty()) {
      continue;
    }
    for (auto j = i + 1; j < kMaxGroup; ++j) {
      if (managers_[j]->empty()) {
        continue;
      }
      if (!filters_[i].test(j)) {
        continue;
      }
      CollisionData collision_data(exclusion_list_);
      if (!end_flag) {
        collision_data.request.num_max_contacts = std::numeric_limits<size_t>::max();
      }
      managers_[i]->collide(managers_[j].get(), &collision_data, &FclCollisionDetector::CollisionFunction);
      if (collision_data.result.isCollision()) {
        if (end_flag) {
          const auto contact = collision_data.result.getContact(0);
          dst_contact_pair.push_back(MakePairString(static_cast<ObjectInfo*>(contact.o1->getUserData())->name,
                                                    static_cast<ObjectInfo*>(contact.o2->getUserData())->name));
          return true;
        } else {
          for (auto x = 0; x < collision_data.result.numContacts(); ++x) {
            dst_contact_pair.push_back(MakePairString(
                static_cast<ObjectInfo*>(collision_data.result.getContact(x).o1->getUserData())->name,
                static_cast<ObjectInfo*>(collision_data.result.getContact(x).o2->getUserData())->name));
          }
        }
      }
    }
  }
  for (auto& pair : additional_list_) {
    if (CheckCollisionPair(pair.first, pair.second)) {
      dst_contact_pair.push_back(MakePairString(pair.first, pair.second));
    }
  }

  std::sort(dst_contact_pair.begin(), dst_contact_pair.end());
  dst_contact_pair.erase(std::unique(dst_contact_pair.begin(), dst_contact_pair.end()), dst_contact_pair.end());
  return !dst_contact_pair.empty();
}

bool FclCollisionDetector::CollisionFunction(fcl::CollisionObjectf* o1, fcl::CollisionObjectf* o2, void* data) {
  assert(data != nullptr);
  auto* collision_data = static_cast<CollisionData*>(data);
  const auto& request = collision_data->request;
  auto& result = collision_data->result;

  if (collision_data->done) return true;

  const auto o1_name = static_cast<ObjectInfo*>(o1->collisionGeometry()->getUserData())->name;
  const auto o2_name = static_cast<ObjectInfo*>(o2->collisionGeometry()->getUserData())->name;
  if (std::find_if(collision_data->exclusion_list_.begin(),
                   collision_data->exclusion_list_.end(),
                   IsEqualPairString({o1_name, o2_name})) != collision_data->exclusion_list_.end()) {
    return false;
  }

  fcl::collide(o1, o2, request, result);

  if (!request.enable_cost && result.isCollision() &&
      result.numContacts() >= request.num_max_contacts) {
    collision_data->done = true;
  }

  return collision_data->done;
}

void FclCollisionDetector::ComputeAABB(CollisionObjectWithInfo& collision_object) {
  if (collision_object.object_info->is_pose_changed) {
    collision_object.collision_object->computeAABBImproved();
    collision_object.object_info->is_pose_changed = false;
    do_manager_update_[collision_object.object_info->group_index] = true;
  }
}

std::optional<fcl::Contactf> FclCollisionDetector::CheckCollisionPairImpl(
    const std::string& nameA, const std::string& nameB) {
  auto itA = collision_objects_.find(nameA);
  if (itA == collision_objects_.end()) {
    throw std::domain_error("CheckCollisionPair error: not exist object name: " + nameA);
  }
  auto itB = collision_objects_.find(nameB);
  if (itB == collision_objects_.end()) {
    throw std::domain_error("CheckCollisionPair error: not exist object name: " + nameB);
  }
  // This is how ODE is designed, and the tests are written based on this assumption, so it can't be helped
  if (nameA == nameB) {
    return std::nullopt;
  }
  auto group_A_index = itA->second.object_info->group_index;
  auto group_B_index = itB->second.object_info->group_index;
  if (group_A_index != group_B_index && !filters_[group_A_index].test(group_B_index)) {
    return std::nullopt;
  }

  ComputeAABB(itA->second);
  ComputeAABB(itB->second);
  if (!itA->second.collision_object->getAABB().overlap(itB->second.collision_object->getAABB())) {
    return std::nullopt;
  }

  // TODO(Takeshita) この実装だと"含まれている"状態の結果が正しくない
  fcl::DefaultCollisionData<float> collision_data;
  collision_data.request.gjk_solver_type = fcl::GST_LIBCCD;
  collision_data.request.enable_contact = true;
  fcl::collide(itA->second.collision_object.get(), itB->second.collision_object.get(),
                collision_data.request, collision_data.result);
  if (collision_data.result.isCollision()) {
    return collision_data.result.getContact(0);
  } else {
    return std::nullopt;
  }
}
}  // namespace tmc_collision_detector
