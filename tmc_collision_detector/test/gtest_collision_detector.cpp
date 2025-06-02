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
 * gtest_coldet.cpp
 *
 *  Created on: 2012/01/16
 *      Author: takeshita
 */
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include "tmc_collision_detector/fcl_collision_detector.hpp"
#include "tmc_collision_detector/ODE_collision_detector.hpp"

// Test whether many objects can be generated
#define CREATEOBJECT_CASE6 0

using tmc_manipulation_types::ObjectParameter;
using tmc_manipulation_types::Shape;
using tmc_manipulation_types::AABB;
using tmc_manipulation_types::CollisionObjectType;
using tmc_manipulation_types::kBox;
using tmc_manipulation_types::kMesh;
using tmc_manipulation_types::kSphere;
using tmc_manipulation_types::kCapsule;
using tmc_manipulation_types::kCylinder;
using tmc_manipulation_types::kMeshVertices;

namespace tmc_collision_detector {
const char* kNode = "gtest_collision_detector";
const char kObjectName[] = "object";
const char kNonExistName[] = "objct";

const char* kTestForlderPath = "test_forlder_path";
const char* kSTLCorrectFileName = "cylinder.stl";
const char* kSTLNothingFileName = "nothing.stl";
const char* kSTLZeroFileName = "zero.stl";
const double kPrimParams[3] = {0.10, 0.20, 0.30};
const double kPrimCoordinate[3] = {-0.10, 0.10, 0.50};
const double kMeshAABB[3] = {0.20, 0.15, 0.20};

const uint16_t kGroup[4] = {1, 2, 4, 8};
const uint16_t kFilter[4] = {14, 13, 11, 7};
const double kSpaceCoordinate[4][2] = {
    {0.0, 0.0},
    {3 * kPrimParams[0], 0.0},
    {0.0, -1.5 * kPrimParams[0]},
    {0.0, 1.5 * kPrimParams[0]}
};
const double kClosestObjectCoordinate[6][2] = {
    {0.0, 0.0},
    {3 * kPrimParams[0], 0.0},
    {-4 * kPrimParams[0], 0.0},
    {-4 * kPrimParams[0], 2 * kPrimParams[0]},
    {-4 * kPrimParams[0], 4 * kPrimParams[0]},
    {10 * kPrimParams[0], 2 * kPrimParams[0]}
};

const double kMargin = 0.02;
const double kEpsilon = 0.005;

enum TestCase {
  kNormalCase1 = 0,
  kNormalCase2,
  kNormalCase3,
  kNormalCase4,
  kNormalCase5,
  kNormalCase6,
  kNormalCase7,
  kNormalCase8,
  kNormalCase9,
  kNormalCase10,
  kAbnormalCase1,
  kAbnormalCase2,
  kAbnormalCase3,
  kAbnormalCase4,
  kAbnormalCase5,
};

class CollisionDetectorTestUtil {
 public:
  virtual ~CollisionDetectorTestUtil() {}

 protected:
  // Functional test for each function
  bool CreateObject_(TestCase case_no);
  bool DestroyObject_(TestCase case_no);
  bool UseAnchor_(TestCase case_no);

  bool SetObjectTransform_(TestCase case_no);
  bool GetObjectTransform_(TestCase case_no);

  bool ChangeObjectPropertyFunctions_(TestCase case_no);

  bool CheckCollision_(enum TestCase case_no);
  bool CheckCollisionHogeHoge_(enum TestCase case_no,
                               CollisionObjectType typeA,
                               CollisionObjectType typeB);
  bool CheckCollisionSpace_(enum TestCase case_no);
  bool GetContactPairList_(enum TestCase case_no);
  bool GetClosestObject_(enum TestCase case_no);
  bool RayCasting_(enum TestCase case_no);
  bool CheckCollisionPair(enum TestCase case_no);

  ObjectParameter InitObjectParameter_(const std::string& name,
                                       CollisionObjectType type);
  void SetObjectDistance_(CollisionObjectType type,
                          Eigen::Vector3d& transform,
                          uint16_t &object_number);

  ICollisionDetector::Ptr coldet_;
  virtual void InitializeCollisionDetector_() = 0;
};

ObjectParameter CollisionDetectorTestUtil::InitObjectParameter_(
    const std::string& name, CollisionObjectType type) {
  ObjectParameter parameter;
  parameter.name = name;
  parameter.margin = kMargin;
  parameter.shape.type = type;
  if (type == kBox) {
    parameter.shape.dimensions.push_back(kPrimParams[0]);
    parameter.shape.dimensions.push_back(kPrimParams[1]);
    parameter.shape.dimensions.push_back(kPrimParams[2]);
  } else if (type == kSphere) {
    parameter.shape.dimensions.push_back(kPrimParams[0]);
  } else if (type == kCapsule) {
    parameter.shape.dimensions.push_back(kPrimParams[0]);
    parameter.shape.dimensions.push_back(kPrimParams[1]);
  } else if (type == kCylinder) {
    parameter.shape.dimensions.push_back(kPrimParams[0]);
    parameter.shape.dimensions.push_back(kPrimParams[1]);
  } else if (type == kMesh) {
    parameter.shape.filename = kSTLCorrectFileName;
  } else if (type == kMeshVertices) {
    std::string dir(__FILE__);
    dir = dir.substr(0, dir.find_last_of("/"));
    const auto cylinder_shape = shapes::createMeshFromResource(std::string("file://") + dir + "/cylinder.stl");
    for (auto i = 0u; i < cylinder_shape->vertex_count; ++i) {
      parameter.shape.vertices.push_back(Eigen::Vector3f(cylinder_shape->vertices[3 * i],
                                                         cylinder_shape->vertices[3 * i + 1],
                                                         cylinder_shape->vertices[3 * i + 2]));
    }
    for (auto i = 0u; i < cylinder_shape->triangle_count; ++i) {
      parameter.shape.indices.push_back(cylinder_shape->triangles[3 * i]);
      parameter.shape.indices.push_back(cylinder_shape->triangles[3 * i + 1]);
      parameter.shape.indices.push_back(cylinder_shape->triangles[3 * i + 2]);
    }
  }
  parameter.transform.setIdentity();
  parameter.group = 0x0001;
  parameter.filter = 0xFFFF;

  return parameter;
}

bool CollisionDetectorTestUtil::CreateObject_(enum TestCase case_no) {
  InitializeCollisionDetector_();

  ObjectParameter parameter;
  switch (case_no) {
    case kNormalCase1:
      parameter = InitObjectParameter_(kObjectName, kSphere);
      break;
    case kNormalCase2:
      parameter = InitObjectParameter_(kObjectName, kBox);
      break;
    case kNormalCase3:
      parameter = InitObjectParameter_(kObjectName, kCapsule);
      break;
    case kNormalCase4:
      parameter = InitObjectParameter_(kObjectName, kCylinder);
      break;
    case kNormalCase5:
      parameter = InitObjectParameter_(kObjectName, kMesh);
      break;
    case kNormalCase6:
      parameter = InitObjectParameter_(kObjectName, kBox);
      break;
    case kNormalCase7:
      parameter = InitObjectParameter_(kObjectName, kMeshVertices);
      break;
    case kAbnormalCase1:
      parameter = InitObjectParameter_(kObjectName, kBox);
      parameter.shape.type = static_cast<CollisionObjectType>(50);
      break;
    case kAbnormalCase2:
      parameter = InitObjectParameter_(kObjectName, kMesh);
      parameter.shape.filename.assign(kSTLNothingFileName);
      break;
    case kAbnormalCase3:
      parameter = InitObjectParameter_(kObjectName, kMesh);
      parameter.shape.filename.assign(kSTLZeroFileName);
      break;
    case kAbnormalCase4:
      parameter = InitObjectParameter_(kObjectName, kBox);
      parameter.shape.dimensions.clear();
      break;
    case kAbnormalCase5:
      parameter = InitObjectParameter_(kObjectName, kSphere);
      parameter.shape.dimensions[0] = -kPrimParams[0];
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }
  try {
    coldet_->CreateObject(parameter);
    // Verify object creation by checking if information can be obtained from the created objects
    const auto param_from_coldet = coldet_->GetObjectParameter(kObjectName);
    EXPECT_EQ(param_from_coldet.name, kObjectName);
    coldet_->CheckCollisionSpace();
#if CREATEOBJECT_CASE6
    if (case_no == kNormalCase6) {
      for (int32_t i = 1; i < 10000; i++) {
        coldet_->CreateObject(parameter);
      }
    }
#endif
  } catch (...) {
    return false;
  }

  return true;
}
bool CollisionDetectorTestUtil::DestroyObject_(enum TestCase case_no) {
  InitializeCollisionDetector_();

  switch (case_no) {
    case kNormalCase1: {
      const auto parameter = InitObjectParameter_(kObjectName, kSphere);
      coldet_->CreateObject(parameter);
      break;
    }
    case kNormalCase2: {
      const auto parameter = InitObjectParameter_(kObjectName, kMesh);
      coldet_->CreateObject(parameter);
      break;
    }
    case kAbnormalCase1:
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }
  try {
    coldet_->DestroyObject(kObjectName);
    coldet_->CheckCollisionSpace();
  } catch (...) {
    return false;
  }
  // Verify object destruction by the inability to obtain object information
  EXPECT_THROW(coldet_->GetObjectParameter(kObjectName), std::domain_error);
  return true;
}
bool CollisionDetectorTestUtil::UseAnchor_(TestCase case_no) {
  InitializeCollisionDetector_();

  try {
    switch (case_no) {
      case kNormalCase4: {
        coldet_->CreateObject(InitObjectParameter_(kObjectName, kBox));
        coldet_->SetAnchor();
        coldet_->CreateObject(InitObjectParameter_(kObjectName + std::to_string(1), kBox));
        coldet_->DestroyObject();

        const auto param_from_coldet = coldet_->GetObjectParameter(kObjectName);
        EXPECT_EQ(param_from_coldet.name, kObjectName);
        EXPECT_THROW(coldet_->GetObjectParameter(kObjectName + std::to_string(1)),
                     std::domain_error);
        break;
      }
      case kAbnormalCase2:
        coldet_->SetAnchor();
        break;
      case kAbnormalCase3:
        coldet_->CreateObject(InitObjectParameter_(kObjectName, kBox));
        coldet_->DestroyObject();
        break;
      default:
        assert(!"Beyond expectation.");
        return false;
    }
    coldet_->CheckCollisionSpace();
  } catch (...) {
    return false;
  }

  return true;
}
bool CollisionDetectorTestUtil::SetObjectTransform_(enum TestCase case_no) {
  InitializeCollisionDetector_();
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kBox);

  Eigen::Affine3d transform;
  transform.translation() =
      Eigen::Vector3d(kPrimCoordinate[0],
                      kPrimCoordinate[1],
                      kPrimCoordinate[2]);
  switch (case_no) {
    case kNormalCase1:
      coldet_->CreateObject(parameter);
      break;
    case kAbnormalCase1:
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }
  try {
    coldet_->SetObjectTransform(transform, kObjectName);
    coldet_->CheckCollisionSpace();
  } catch (...) {
    return false;
  }

  return true;
}
bool CollisionDetectorTestUtil::GetObjectTransform_(enum TestCase case_no) {
  InitializeCollisionDetector_();
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kBox);

  Eigen::Affine3d transform;
  transform.translation() =
      Eigen::Vector3d(kPrimCoordinate[0],
                      kPrimCoordinate[1],
                      kPrimCoordinate[2]);
  switch (case_no) {
    case kNormalCase1:
      coldet_->CreateObject(parameter);
      coldet_->SetObjectTransform(transform, kObjectName);
      break;
    case kAbnormalCase1:
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }

  Eigen::Affine3d get_transform;
  try {
    get_transform = coldet_->GetObjectTransform(kObjectName);
    coldet_->CheckCollisionSpace();
  } catch (...) {
    return false;
  }
  if ((fabs(get_transform.translation().x() - kPrimCoordinate[0]) < kEpsilon) &&
      (fabs(get_transform.translation().y() - kPrimCoordinate[1]) < kEpsilon) &&
      (fabs(get_transform.translation().z() - kPrimCoordinate[2]) < kEpsilon)) {
    return true;
  } else {
    return false;
  }
}
bool CollisionDetectorTestUtil::ChangeObjectPropertyFunctions_(TestCase case_no) {
  InitializeCollisionDetector_();
  uint16_t parameter = 1;

  try {
    switch (case_no) {
      case kNormalCase1:
        coldet_->CreateObject(InitObjectParameter_(kObjectName, kBox));
        EXPECT_EQ(coldet_->GetCollisionGroup(kObjectName), 0x0001);
        coldet_->SetCollisionGroup(2, kObjectName);
        EXPECT_EQ(coldet_->GetCollisionGroup(kObjectName), 0x0002);
        break;
      case kNormalCase2:
        coldet_->CreateObject(InitObjectParameter_(kObjectName, kBox));
        EXPECT_EQ(coldet_->GetCollisionFilter(kObjectName), 0xFFFF);
        coldet_->SetCollisionFilter(0xFFFE, kObjectName);
        EXPECT_EQ(coldet_->GetCollisionFilter(kObjectName), 0xFFFE);
        break;
      case kAbnormalCase1:
        coldet_->SetCollisionGroup(parameter, kObjectName);
        break;
      case kAbnormalCase2:
        coldet_->SetCollisionFilter(parameter, kObjectName);
        break;
      case kAbnormalCase3:
        coldet_->EnableObject(kObjectName);
        break;
      case kAbnormalCase4:
        coldet_->DisableObject(kObjectName);
        break;
      default:
        assert(!"Beyond expectation.");
        return false;
    }
    coldet_->CheckCollisionSpace();
  } catch (...) {
    return false;
  }

  return true;
}

bool CollisionDetectorTestUtil::CheckCollision_(enum TestCase case_no) {
  InitializeCollisionDetector_();
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kBox);
  coldet_->CreateObject(parameter);

  try {
    switch (case_no) {
      case kAbnormalCase1:
        coldet_->CheckCollisionPair(kNonExistName, kObjectName);
        break;
      case kAbnormalCase2:
        coldet_->CheckCollisionPair(kObjectName, kNonExistName);
        break;
      default:
        assert(!"Beyond expectation.");
        return false;
    }
  } catch (...) {
    return false;
  }

  return true;
}

bool CollisionDetectorTestUtil::CheckCollisionHogeHoge_(
    enum TestCase case_no,
    CollisionObjectType typeA,
    CollisionObjectType typeB) {
  InitializeCollisionDetector_();

  // Sphere
  coldet_->CreateObject(InitObjectParameter_(kObjectName + std::to_string(0), kSphere));
  coldet_->CreateObject(InitObjectParameter_(kObjectName + std::to_string(6), kSphere));

  // Box
  coldet_->CreateObject(InitObjectParameter_(kObjectName + std::to_string(1), kBox));
  coldet_->CreateObject(InitObjectParameter_(kObjectName + std::to_string(7), kBox));

  // Capsule
  coldet_->CreateObject(InitObjectParameter_(kObjectName + std::to_string(2), kCapsule));
  coldet_->CreateObject(InitObjectParameter_(kObjectName + std::to_string(8), kCapsule));

  // Cylinder
  coldet_->CreateObject(InitObjectParameter_(kObjectName + std::to_string(3), kCylinder));
  coldet_->CreateObject(InitObjectParameter_(kObjectName + std::to_string(9), kCylinder));

  // Mesh
  coldet_->CreateObject(InitObjectParameter_(kObjectName + std::to_string(4), kMesh));
  coldet_->CreateObject(InitObjectParameter_(kObjectName + std::to_string(10), kMesh));

  // Mesh (vertex specified)
  coldet_->CreateObject(InitObjectParameter_(kObjectName + std::to_string(5), kMeshVertices));
  coldet_->CreateObject(InitObjectParameter_(kObjectName + std::to_string(11), kMeshVertices));

  Eigen::Vector3d translation(0.0, 0.0, 0.0);
  uint16_t object_number[2] = {0, 0};
  SetObjectDistance_(typeA, translation, object_number[0]);
  SetObjectDistance_(typeB, translation, object_number[1]);

  std::string hogeA_name = kObjectName + std::to_string(object_number[0]);
  std::string hogeB_name = kObjectName + std::to_string(object_number[1] + 6);

  Eigen::Affine3d object_transform;
  object_transform.setIdentity();
  bool expect_result = false;
  switch (case_no) {
    case kNormalCase1:
      object_transform.translation() =
          Eigen::Vector3d(translation.x() + 3 * kMargin, 0.0, 0.0);
      break;
    case kNormalCase2:
      object_transform.translation() =
          Eigen::Vector3d(translation.x() + kMargin, 0.0, 0.0);
      break;
    case kNormalCase3:
      object_transform.translation() =
          Eigen::Vector3d(translation.x() - kMargin, 0.0, 0.0);
      expect_result = true;
      break;
    case kNormalCase4:
      object_transform.translation() =
          Eigen::Vector3d(0.0, translation.y() + 3 * kMargin, 0.0);
      break;
    case kNormalCase5:
      object_transform.translation() =
          Eigen::Vector3d(0.0, translation.y() + kMargin, 0.0);
      break;
    case kNormalCase6:
      object_transform.translation() =
          Eigen::Vector3d(0.0, translation.y() - kMargin, 0.0);
      expect_result = true;
      break;
    case kNormalCase7:
      object_transform.translation() =
          Eigen::Vector3d(0.0, 0.0, translation.z() + 3 * kMargin);
      break;
    case kNormalCase8:
      object_transform.translation() =
          Eigen::Vector3d(0.0, 0.0, translation.z() + kMargin);
      break;
    case kNormalCase9:
      object_transform.translation() =
          Eigen::Vector3d(0.0, 0.0, translation.z() - kMargin);
      expect_result = true;
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }

  coldet_->SetObjectTransform(object_transform, hogeA_name);

  bool coldet_result = coldet_->CheckCollisionPair(hogeA_name, hogeB_name);
  if (coldet_result == expect_result) {
    return true;
  }

  return false;
}
bool CollisionDetectorTestUtil::CheckCollisionSpace_(enum TestCase case_no) {
  InitializeCollisionDetector_();

  ObjectParameter parameter = InitObjectParameter_(kObjectName, kSphere);
  for (int32_t i = 0; i < 4; i++) {
    parameter.name = "sphere" + std::to_string(i);
    parameter.filter = kFilter[i];
    parameter.group = kGroup[i];
    parameter.transform.translation() =
        Eigen::Vector3d(kSpaceCoordinate[i][0], kSpaceCoordinate[i][1], 0.0);
    coldet_->CreateObject(parameter);
  }
  std::vector<PairString> disable_pairs;
  std::vector<PairString> enable_pairs;
  bool expect_result = false;
  switch (case_no) {
    case kNormalCase1:
      expect_result = true;
      break;
    case kNormalCase2:
      coldet_->DisableObject(std::string("sphere0"));
      break;
    case kNormalCase3:
      coldet_->DisableObject(std::string("sphere0"));
      coldet_->EnableObject(std::string("sphere0"));
      expect_result = true;
      break;
    case kNormalCase4:
      coldet_->SetCollisionGroup(kGroup[3], std::string("sphere0"));
      coldet_->SetCollisionFilter(kFilter[3], std::string("sphere0"));
      expect_result = true;
      break;
    case kNormalCase5:
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere0"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere0"));
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere3"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere3"));
      break;
    case kNormalCase6:
      coldet_->DestroyObject(std::string("sphere0"));
      coldet_->DestroyObject(std::string("sphere1"));
      coldet_->DestroyObject(std::string("sphere2"));
      coldet_->DestroyObject(std::string("sphere3"));
      break;
    case kNormalCase7:
      disable_pairs.push_back(PairString("sphere0", "sphere2"));
      disable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->DisableCollisionCheck(disable_pairs);
      break;
    case kNormalCase8:
      disable_pairs.push_back(PairString("sphere0", "sphere2"));
      disable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->DisableCollisionCheck(disable_pairs);
      coldet_->ResetCollisionCheckPairList();
      expect_result = true;
      break;
    case kNormalCase9:
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere0"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere0"));
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere3"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere3"));
      enable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->EnableCollisionCheck(enable_pairs);
      expect_result = true;
      break;
    case kNormalCase10:
      disable_pairs.push_back(PairString("sphere0", "sphere2"));
      coldet_->DisableCollisionCheck(disable_pairs);
      enable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->EnableCollisionCheck(enable_pairs);
      coldet_->DestroyObject(std::string("sphere0"));
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }

  PairString pair_name;
  bool check_space_result = coldet_->CheckCollisionSpace(pair_name);
  if (check_space_result == expect_result) {
    return true;
  }

  return false;
}
bool CollisionDetectorTestUtil::GetContactPairList_(enum TestCase case_no) {
  InitializeCollisionDetector_();
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kSphere);

  for (int32_t i = 0; i < 4; i++) {
    parameter.name = "sphere" + std::to_string(i);
    parameter.filter = kFilter[i];
    parameter.group = kGroup[i];
    parameter.transform.translation() =
        Eigen::Vector3d(kSpaceCoordinate[i][0], kSpaceCoordinate[i][1], 0.0);
    coldet_->CreateObject(parameter);
  }

  std::vector<PairString> disable_pairs;
  std::vector<PairString> enable_pairs;
  uint32_t expect_contact_num = 0;
  bool expect_result = false;
  switch (case_no) {
    case kNormalCase1:
      expect_contact_num = 2;
      expect_result = true;
      break;
    case kNormalCase2:
      coldet_->DisableObject(std::string("sphere0"));
      break;
    case kNormalCase3:
      coldet_->DisableObject(std::string("sphere0"));
      coldet_->EnableObject(std::string("sphere0"));
      expect_contact_num = 2;
      expect_result = true;
      break;
    case kNormalCase4:
      coldet_->SetCollisionGroup(kGroup[3], std::string("sphere0"));
      coldet_->SetCollisionFilter(kFilter[3], std::string("sphere0"));
      expect_contact_num = 1;
      expect_result = true;
      break;
    case kNormalCase5:
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere0"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere0"));
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere3"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere3"));
      break;
    case kNormalCase6:
      coldet_->DestroyObject(std::string("sphere0"));
      coldet_->DestroyObject(std::string("sphere1"));
      coldet_->DestroyObject(std::string("sphere2"));
      coldet_->DestroyObject(std::string("sphere3"));
      break;
    case kNormalCase7:
      disable_pairs.push_back(PairString("sphere0", "sphere2"));
      disable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->DisableCollisionCheck(disable_pairs);
      break;
    case kNormalCase8:
      disable_pairs.push_back(PairString("sphere0", "sphere2"));
      disable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->DisableCollisionCheck(disable_pairs);
      enable_pairs.push_back(PairString("sphere0", "sphere2"));
      coldet_->EnableCollisionCheck(enable_pairs);
      expect_contact_num = 1;
      expect_result = true;
      break;
    case kNormalCase9:
      disable_pairs.push_back(PairString("sphere0", "sphere2"));
      disable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->DisableCollisionCheck(disable_pairs);
      coldet_->ResetCollisionCheckPairList();
      expect_contact_num = 2;
      expect_result = true;
      break;
    case kNormalCase10:
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere0"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere0"));
      coldet_->SetCollisionGroup(kGroup[2], std::string("sphere3"));
      coldet_->SetCollisionFilter(kFilter[2], std::string("sphere3"));
      enable_pairs.push_back(PairString("sphere0", "sphere3"));
      coldet_->EnableCollisionCheck(enable_pairs);
      expect_contact_num = 1;
      expect_result = true;
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }

  std::vector<PairString> pair_name_list;
  bool check_space_result = coldet_->GetContactPairList(pair_name_list);
  if ((check_space_result == expect_result) &&
      (expect_contact_num == pair_name_list.size())) {
    return true;
  }
  return false;
}
bool CollisionDetectorTestUtil::GetClosestObject_(enum TestCase case_no) {
  InitializeCollisionDetector_();
  ObjectParameter parameter = InitObjectParameter_(kObjectName, kSphere);

  for (int32_t i = 0; i < 6; i++) {
    parameter.name = "sphere" + std::to_string(i);
    parameter.filter = 3;
    parameter.group = 4;
    parameter.transform.translation() =
        Eigen::Vector3d(kClosestObjectCoordinate[i][0],
                        kClosestObjectCoordinate[i][1], 0.0);
    coldet_->CreateObject(parameter);
  }
  coldet_->SetCollisionFilter(4, std::string("sphere0"));
  coldet_->SetCollisionGroup(1, std::string("sphere0"));
  coldet_->SetCollisionFilter(4, std::string("sphere1"));
  coldet_->SetCollisionGroup(2, std::string("sphere1"));

  int32_t top_n = 3;
  double extend = 5.0 * kPrimParams[0];
  uint16_t filter = 4;
  std::string expect_answer;
  switch (case_no) {
    case kNormalCase1:
      expect_answer.assign("sphere2");
      break;
    case kNormalCase2:
      expect_answer.assign("sphere3");
      coldet_->DisableObject(std::string("sphere2"));
      break;
    case kNormalCase3:
      expect_answer.assign("");
      extend = 0.5 * kPrimParams[0];
      break;
    case kNormalCase4:
      expect_answer.assign("");
      coldet_->DisableObject(std::string("sphere2"));
      coldet_->DisableObject(std::string("sphere3"));
      coldet_->DisableObject(std::string("sphere4"));
      break;
    case kNormalCase5:
      expect_answer.assign("sphere");
      top_n = 1;
      break;
    case kNormalCase6:
      expect_answer.assign("sphere0");
      coldet_->DisableObject(std::string("sphere0"));
      break;
    case kAbnormalCase1:
      top_n = 0;
      break;
    case kAbnormalCase2:
      extend = 0.0;
      break;
    case kAbnormalCase3:
      filter = 0;
      break;
    case kAbnormalCase4:
      coldet_->DestroyObject(std::string("sphere0"));
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }
  try {
    ClosestResult result =
        coldet_->GetClosestObject(std::string("sphere0"),
                                  extend, top_n, filter);
    if (result.name.npos == result.name.find(expect_answer)) {
      return false;
    }
  } catch (...) {
    return false;
  }
  return true;
}
bool CollisionDetectorTestUtil::RayCasting_(enum TestCase case_no) {
  InitializeCollisionDetector_();

  Eigen::Vector3d start_point(Eigen::Vector3d::Zero());
  Eigen::Vector3d direction(Eigen::Vector3d::Random());
  direction.normalize();
  double length = 1.0;

  bool expect_result = true;

  ObjectParameter parameter;
  switch (case_no) {
    case kNormalCase1:
      parameter = InitObjectParameter_(kObjectName, kSphere);
      parameter.transform.translation() = direction;
      break;
    case kNormalCase2:
      parameter = InitObjectParameter_(kObjectName, kBox);
      parameter.transform.translation() = direction;
      break;
    case kNormalCase3:
      parameter = InitObjectParameter_(kObjectName, kCapsule);
      parameter.transform.translation() = direction;
      break;
    case kNormalCase4:
      parameter = InitObjectParameter_(kObjectName, kCylinder);
      parameter.transform.translation() = direction;
      break;
    case kNormalCase5:
      parameter = InitObjectParameter_(kObjectName, kMesh);
      parameter.transform.translation() = direction;
      break;
    case kNormalCase6:
      parameter = InitObjectParameter_(kObjectName, kSphere);
      parameter.transform.translation() = direction * (-1.0);
      expect_result = false;
      break;
    case kNormalCase7:
      parameter = InitObjectParameter_(kObjectName, kSphere);
      parameter.transform.translation() = direction * 5.0;
      expect_result = false;
      break;
    case kNormalCase8:
      parameter = InitObjectParameter_(kObjectName, kSphere);
      parameter.transform.translation() = direction;
      expect_result = false;
      break;
    case kAbnormalCase1:
      parameter = InitObjectParameter_(kObjectName, kSphere);
      parameter.transform.translation() = direction;
      direction.setZero(3);
      break;
    case kAbnormalCase2:
      parameter = InitObjectParameter_(kObjectName, kSphere);
      parameter.transform.translation() = direction;
      length = 0.0;
      break;
    default:
      assert(!"Beyond expectation.");
      return false;
  }

  coldet_->CreateObject(parameter);
  try {
    Eigen::Vector3d end_point;
    std::string name;
    if (case_no == kNormalCase8) {
      coldet_->DisableObject(kObjectName);
    }
    bool result =
        coldet_->RayCasting(start_point, direction, length, end_point, name);
    if (result == expect_result) {
      return true;
    } else {
      return false;
    }
  } catch (...) {
    return false;
  }
}

void CollisionDetectorTestUtil::SetObjectDistance_(
    CollisionObjectType type,
    Eigen::Vector3d& transform,
    uint16_t &object_number) {
  switch (type) {
    case kSphere:
      object_number = 0;
      transform += Eigen::Vector3d(kPrimParams[0],
                                   kPrimParams[0],
                                   kPrimParams[0]);
      break;
    case kBox:
      object_number = 1;
      transform += Eigen::Vector3d(kPrimParams[0] / 2.0,
                                   kPrimParams[1] / 2.0,
                                   kPrimParams[2] / 2.0);
      break;
    case kCapsule:
      object_number = 2;
      transform += Eigen::Vector3d(kPrimParams[0],
                                   kPrimParams[0],
                                   kPrimParams[0] + kPrimParams[1] / 2.0);
      break;
    case kCylinder:
      object_number = 3;
      transform += Eigen::Vector3d(kPrimParams[0],
                                   kPrimParams[0],
                                   kPrimParams[1] / 2.0);
      break;
    case kMesh:
      object_number = 4;
      transform += Eigen::Vector3d(kMeshAABB[0], kMeshAABB[1], kMeshAABB[2]);
      break;
    case kMeshVertices:
      object_number = 5;
      transform += Eigen::Vector3d(kMeshAABB[0], kMeshAABB[1], kMeshAABB[2]);
      break;
    default:
      assert(!"Beyond expectation.");
      return;
  }
}

template<typename CollisionDetectorType>
class CollisionDetectorTest : public ::testing::Test, public CollisionDetectorTestUtil {
 protected:
  void InitializeCollisionDetector_() override {
    coldet_ = std::make_shared<CollisionDetectorType>();
  }
};

using Implementations = ::testing::Types<ODECollisionDetector, FclCollisionDetector>;
TYPED_TEST_SUITE(CollisionDetectorTest, Implementations);

TYPED_TEST(CollisionDetectorTest, CreateObject) {
  // kNormalCase1: Create a sphere
  // kNormalCase2: Create a box
  // kNormalCase3: Create a capsule
  // kNormalCase4: Create a cylinder
  // kNormalCase5: Create a mesh
  // kNormalCase6: Create 10000 boxes
  // kNormalCase7: Create a mesh (vertex specified)

  // kAbnormalCase1: Invalid type
  // kAbnormalCase2: STL file does not exist
  // kAbnormalCase3: STL file is empty
  // kAbnormalCase4: Box parameters do not exist
  // kAbnormalCase5: Sphere parameter is negative
  EXPECT_TRUE(this->CreateObject_(kNormalCase1));
  EXPECT_TRUE(this->CreateObject_(kNormalCase2));
  EXPECT_TRUE(this->CreateObject_(kNormalCase3));
  EXPECT_TRUE(this->CreateObject_(kNormalCase4));
  EXPECT_TRUE(this->CreateObject_(kNormalCase5));
  EXPECT_TRUE(this->CreateObject_(kNormalCase6));
  EXPECT_TRUE(this->CreateObject_(kNormalCase7));
  EXPECT_FALSE(this->CreateObject_(kAbnormalCase1));
  EXPECT_FALSE(this->CreateObject_(kAbnormalCase2));
  EXPECT_FALSE(this->CreateObject_(kAbnormalCase3));
  EXPECT_FALSE(this->CreateObject_(kAbnormalCase4));
  EXPECT_FALSE(this->CreateObject_(kAbnormalCase5));
}

TYPED_TEST(CollisionDetectorTest, DestroyObject) {
  // kNormalCase1: Create and destroy a primitive
  // kNormalCase2: Create and destroy a mesh
  // kAbnormalCase1: Destroy an object that was not created
  EXPECT_TRUE(this->DestroyObject_(kNormalCase1));
  EXPECT_TRUE(this->DestroyObject_(kNormalCase2));
  EXPECT_FALSE(this->DestroyObject_(kAbnormalCase1));
}

TYPED_TEST(CollisionDetectorTest, UseAnchor) {
  // kNormalCase4: Set the created primitive as an anchor and use the anchor to destroy the object
  // kAbnormalCase2: Set anchor without creating a primitive
  // kAbnormalCase3: Use anchor to destroy object without setting anchor

  EXPECT_TRUE(this->UseAnchor_(kNormalCase4));
  EXPECT_FALSE(this->UseAnchor_(kAbnormalCase2));
  EXPECT_FALSE(this->UseAnchor_(kAbnormalCase3));
}

TYPED_TEST(CollisionDetectorTest, SetObjectTransform) {
  // kNormalCase1: Normal case
  // kAbnormalCase1: Set coordinates for an object not created
  EXPECT_TRUE(this->SetObjectTransform_(kNormalCase1));
  EXPECT_FALSE(this->SetObjectTransform_(kAbnormalCase1));
}

TYPED_TEST(CollisionDetectorTest, GetObjectTransform) {
  // kNormalCase1: Normal case
  // kAbnormalCase1: Get coordinates of an object not created

  EXPECT_TRUE(this->GetObjectTransform_(kNormalCase1));
  EXPECT_FALSE(this->GetObjectTransform_(kAbnormalCase1));
}

TYPED_TEST(CollisionDetectorTest, ChangeObjectPropertyFunctions) {
  // Fail test for setting category/filter and enabling/disabling object functions
  // kNormalCase1: Normal case for Set/GetCollisionGroup
  // kNormalCase2: Normal case for Set/GetCollisionFilter
  // kAbnormalCase1: SetCollisionGroup() for an object not created
  // kAbnormalCase2: SetCollisionFilter() for an object not created
  // kAbnormalCase3: EnableObject() for an object not created
  // kAbnormalCase4: DisableObject() for an object not created

  EXPECT_TRUE(this->ChangeObjectPropertyFunctions_(kNormalCase1));
  EXPECT_TRUE(this->ChangeObjectPropertyFunctions_(kNormalCase2));
  EXPECT_FALSE(this->ChangeObjectPropertyFunctions_(kAbnormalCase1));
  EXPECT_FALSE(this->ChangeObjectPropertyFunctions_(kAbnormalCase2));
  EXPECT_FALSE(this->ChangeObjectPropertyFunctions_(kAbnormalCase3));
  EXPECT_FALSE(this->ChangeObjectPropertyFunctions_(kAbnormalCase4));
}

TYPED_TEST(CollisionDetectorTest, CheckCollision) {
  // kAbnormalCase1: Interference check for an object not created
  // kAbnormalCase2: Interference check for an object not created

  EXPECT_FALSE(this->CheckCollision_(kAbnormalCase1));
  EXPECT_FALSE(this->CheckCollision_(kAbnormalCase2));
}

// Test if the result of the interference check matches the expected value
// The distance between two objects
// 1. X-axis direction, contact distance + kMargin * 3 (no contact)
// 2. X-axis direction, contact distance + kMargin (no contact)
// 3. X-axis direction, contact distance - kMargin (contact)
// 4. Y-axis direction, contact distance + kMargin * 3 (no contact)
// 5. Y-axis direction, contact distance + kMargin (no contact)
// 6. Y-axis direction, contact distance - kMargin (contact)
// 7. Z-axis direction, contact distance + kMargin * 3 (no contact)
// 8. Z-axis direction, contact distance + kMargin (no contact)
// 9. Z-axis direction, contact distance - kMargin (contact)
// Is as follows.
TYPED_TEST(CollisionDetectorTest, ColDetSphereSphere) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kSphere, kSphere));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kSphere, kSphere));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kSphere, kSphere));
}
TYPED_TEST(CollisionDetectorTest, ColDetSphereBox) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kSphere, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kSphere, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kSphere, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase4, kSphere, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase5, kSphere, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase6, kSphere, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kSphere, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kSphere, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kSphere, kBox));
}
TYPED_TEST(CollisionDetectorTest, ColDetSphereCapsule) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kSphere, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kSphere, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kSphere, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kSphere, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kSphere, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kSphere, kCapsule));
}
TYPED_TEST(CollisionDetectorTest, ColDetSphereCylinder) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kSphere, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kSphere, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kSphere, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kSphere, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kSphere, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kSphere, kCylinder));
}
TYPED_TEST(CollisionDetectorTest, ColDetSphereMesh) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kSphere, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kSphere, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kSphere, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase4, kSphere, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase5, kSphere, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase6, kSphere, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kSphere, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kSphere, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kSphere, kMesh));
}
TYPED_TEST(CollisionDetectorTest, ColDetSphereMeshVertices) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kSphere, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kSphere, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kSphere, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase4, kSphere, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase5, kSphere, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase6, kSphere, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kSphere, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kSphere, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kSphere, kMeshVertices));
}
TYPED_TEST(CollisionDetectorTest, ColDetBoxBox) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kBox, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kBox, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kBox, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase4, kBox, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase5, kBox, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase6, kBox, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kBox, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kBox, kBox));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kBox, kBox));
}
TYPED_TEST(CollisionDetectorTest, ColDetBoxCapsule) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kBox, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kBox, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kBox, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kBox, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kBox, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kBox, kCapsule));
}
TYPED_TEST(CollisionDetectorTest, ColDetBoxCylinder) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kBox, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kBox, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kBox, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kBox, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kBox, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kBox, kCylinder));
}
TYPED_TEST(CollisionDetectorTest, ColDetBoxMesh) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kBox, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kBox, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kBox, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase4, kBox, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase5, kBox, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase6, kBox, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kBox, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kBox, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kBox, kMesh));
}
TYPED_TEST(CollisionDetectorTest, ColDetBoxMeshVertices) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kBox, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kBox, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kBox, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase4, kBox, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase5, kBox, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase6, kBox, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kBox, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kBox, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kBox, kMeshVertices));
}
TYPED_TEST(CollisionDetectorTest, ColDetCapsuleCapsule) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kCapsule, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kCapsule, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kCapsule, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kCapsule, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kCapsule, kCapsule));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kCapsule, kCapsule));
}
TYPED_TEST(CollisionDetectorTest, ColDetCapsuleCylinder) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kCapsule, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kCapsule, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kCapsule, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kCapsule, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kCapsule, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kCapsule, kCylinder));
}
TYPED_TEST(CollisionDetectorTest, ColDetCapsuleMesh) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kCapsule, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kCapsule, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kCapsule, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kCapsule, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kCapsule, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kCapsule, kMesh));
}
TYPED_TEST(CollisionDetectorTest, ColDetCapsuleMeshVertices) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kCapsule, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kCapsule, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kCapsule, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kCapsule, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kCapsule, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kCapsule, kMeshVertices));
}
TYPED_TEST(CollisionDetectorTest, ColDetCylinderCylinder) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kCylinder, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kCylinder, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kCylinder, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kCylinder, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kCylinder, kCylinder));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kCylinder, kCylinder));
}
TYPED_TEST(CollisionDetectorTest, ColDetCylinderMesh) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kCylinder, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kCylinder, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kCylinder, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kCylinder, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kCylinder, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kCylinder, kMesh));
}
TYPED_TEST(CollisionDetectorTest, ColDetCylinderMeshVertices) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kCylinder, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kCylinder, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kCylinder, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kCylinder, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kCylinder, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kCylinder, kMeshVertices));
}
TYPED_TEST(CollisionDetectorTest, ColDetMeshMesh) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kMesh, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kMesh, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kMesh, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase4, kMesh, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase5, kMesh, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase6, kMesh, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kMesh, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kMesh, kMesh));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kMesh, kMesh));
}
TYPED_TEST(CollisionDetectorTest, ColDetMeshMeshVertices) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kMesh, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kMesh, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kMesh, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase4, kMesh, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase5, kMesh, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase6, kMesh, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kMesh, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kMesh, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kMesh, kMeshVertices));
}
TYPED_TEST(CollisionDetectorTest, ColDetMeshVerticesMeshVertices) {
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase1, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase2, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase3, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase4, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase5, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase6, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase7, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase8, kMeshVertices, kMeshVertices));
  EXPECT_TRUE(this->CheckCollisionHogeHoge_(kNormalCase9, kMeshVertices, kMeshVertices));
}
TYPED_TEST(CollisionDetectorTest, CheckCollisionSpace) {
  // Initial setup:
  // Arrange four spheres of radius r such that their centers are on the xy-plane
  // Set all to different groups
  // No interference with own group
  // Sphere 1 center (0, 0)
  // Sphere 2 center (3*r, 0)
  // Sphere 3 center (0, -1.5*r)
  // Sphere 4 center (0, 1.5*r)
  // kNormalCase1: Do not alter any settings, interference will occur
  // kNormalCase2: Disable sphere 1, no interference
  // kNormalCase3: After disabling sphere 1, enable it again, interference will occur
  // kNormalCase4: Set the group and filter of sphere 1 the same as sphere 3, interference will occur
  // kNormalCase5: Set the group and filter of spheres 1 and 4 the same as sphere 3, no interference
  // kNormalCase6: Destroy spheres 1, 2, 3, 4, no interference
  // kNormalCase7: Exclude sphere 1 and 4, sphere 1 and 3 from interference check, no interference
  // kNormalCase8: After excluding sphere 1 and 4, sphere 1 and 3 from interference check, discard exclusion list, interference will occur
  // kNormalCase9: Set the group and filter of spheres 1, 4 the same as sphere 3 and add to interference check pair, interference will occur

  EXPECT_TRUE(this->CheckCollisionSpace_(kNormalCase1));
  EXPECT_TRUE(this->CheckCollisionSpace_(kNormalCase2));
  EXPECT_TRUE(this->CheckCollisionSpace_(kNormalCase3));
  EXPECT_TRUE(this->CheckCollisionSpace_(kNormalCase4));
  EXPECT_TRUE(this->CheckCollisionSpace_(kNormalCase5));
  EXPECT_TRUE(this->CheckCollisionSpace_(kNormalCase6));
  EXPECT_TRUE(this->CheckCollisionSpace_(kNormalCase7));
  EXPECT_TRUE(this->CheckCollisionSpace_(kNormalCase8));
  EXPECT_TRUE(this->CheckCollisionSpace_(kNormalCase9));
  EXPECT_TRUE(this->CheckCollisionSpace_(kNormalCase10));
}
TYPED_TEST(CollisionDetectorTest, GetContactPairList) {
  // Initial setup:
  // Arrange four spheres of radius r such that their centers are on the xy-plane
  // Set all to different groups
  // No interference with own group
  // Sphere 1 center (0, 0)
  // Sphere 2 center (3*r, 0)
  // Sphere 3 center (0, -1.5*r)
  // Sphere 4 center (0, 1.5*r)
  // kNormalCase1: Do not alter any settings, interference occurs (2 locations)
  // kNormalCase2: Disable sphere 1, no interference
  // kNormalCase3: After disabling sphere 1, enable it again, interference occurs (2 locations)
  // kNormalCase4: Set the group and filter of sphere 1 the same as sphere 3, interference occurs (1 location)
  // kNormalCase5: Set the group and filter of spheres 1, 4 the same as sphere 3, no interference
  // kNormalCase6: Destroy spheres 1, 2, 3, 4, no interference
  // kNormalCase7: Exclude sphere 1 and 4, sphere 1 and 3 from interference check, no interference
  // kNormalCase8: After excluding sphere 1 and 4, sphere 1 and 3 from interference check, enable check for sphere 1 and 3, interference occurs
  // kNormalCase9: After excluding sphere 1 and 4, sphere 1 and 3 from interference check, discard exclusion list, interference occurs
  // kNormalCase10: Set the group and filter of spheres 1, 4 the same as sphere 3 and add to interference check pair, interference occurs

  EXPECT_TRUE(this->GetContactPairList_(kNormalCase1));
  EXPECT_TRUE(this->GetContactPairList_(kNormalCase2));
  EXPECT_TRUE(this->GetContactPairList_(kNormalCase3));
  EXPECT_TRUE(this->GetContactPairList_(kNormalCase4));
  EXPECT_TRUE(this->GetContactPairList_(kNormalCase5));
  EXPECT_TRUE(this->GetContactPairList_(kNormalCase6));
  EXPECT_TRUE(this->GetContactPairList_(kNormalCase7));
  EXPECT_TRUE(this->GetContactPairList_(kNormalCase8));
  EXPECT_TRUE(this->GetContactPairList_(kNormalCase9));
  EXPECT_TRUE(this->GetContactPairList_(kNormalCase10));
}
TYPED_TEST(CollisionDetectorTest, GetClosestObject) {
  // Initial setup:
  // Arrange a sphere of radius r such that its center is on the xy-plane
  // Execute with sphere 1, filter 100
  // Sphere 1: Center (0, 0) Category 001 Filter 100
  // Sphere 2: Center (3 * r, 0) Category 010 Filter 100
  // Sphere 3: Center (- 4 * r, 0) Category 100 Filter 011
  // Sphere 4: Center (- 4 * r, 2 * r) Category 100 Filter 011
  // Sphere 5: Center (- 4 * r, 4 * r) Category 100 Filter 011
  // Sphere 6: Center (10 * r, 2 * r) Category 100 Filter 011

  // ５
  // ４            ６
  // ３　１ ２

  // kNormalCase1: Top 3, expansion 5*r → Sphere 3
  // kNormalCase2: Disable sphere 3, top 3, expansion 5*r → Sphere 4
  // kNormalCase3: Top 3, expansion 0.5 * r → Not found
  // kNormalCase4: Disable spheres 3, 4, 5, top 3, expansion 5*r → Not found
  // kNormalCase5: Top 1, expansion 5*r → Something is found, hopefully sphere 3
  // kNormalCase6: Execute after disabling sphere 1 → Do nothing
  // kAbnormalCase1: top 0
  // kAbnormalCase2: Expansion 0
  // kAbnormalCase3: Filter 0
  // kAbnormalCase4: Execute after destroying sphere 1

  EXPECT_FALSE(this->GetClosestObject_(kNormalCase1));
}

class ODECollisionDetectorTest : public CollisionDetectorTest<ODECollisionDetector> {};

TEST_F(ODECollisionDetectorTest, RayCasting) {
  // kNormalCase1: Target sphere
  // kNormalCase2: Target box
  // kNormalCase3: Target capsule
  // kNormalCase4: Target cylinder
  // kNormalCase5: Target mesh
  // kNormalCase6: No object in the direction of the ray
  // kNormalCase7: Object exists in the direction of the ray, but far
  // kNormalCase8: Disable object

  EXPECT_TRUE(RayCasting_(kNormalCase1));
  EXPECT_TRUE(RayCasting_(kNormalCase2));
  EXPECT_TRUE(RayCasting_(kNormalCase3));
  EXPECT_TRUE(RayCasting_(kNormalCase5));
  EXPECT_TRUE(RayCasting_(kNormalCase6));
  EXPECT_TRUE(RayCasting_(kNormalCase7));
  EXPECT_TRUE(RayCasting_(kNormalCase8));
  EXPECT_FALSE(RayCasting_(kAbnormalCase1));
  EXPECT_FALSE(RayCasting_(kAbnormalCase2));
}

TEST_F(ODECollisionDetectorTest, GetMeshObjectParameter) {
  EXPECT_TRUE(CreateObject_(kNormalCase5));
  const auto param = coldet_->GetObjectParameter(kObjectName);
  // cylinder.stl has 50 vertices, 96 faces
  EXPECT_EQ(param.shape.vertices.size(), 50);
  EXPECT_EQ(param.shape.indices.size(), 96 * 3);
}
}  // namespace tmc_collision_detector

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
