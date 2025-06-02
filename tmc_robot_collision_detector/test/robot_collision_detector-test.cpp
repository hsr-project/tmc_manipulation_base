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
/// @file     robot_collision_detector-test.cpp
/// @brief    Test class that performs interference checking using a robot model
#include <fstream>
#include <string>
#include <vector>

#include "tmc_robot_collision_detector/robot_collision_detector.hpp"

#include <gtest/gtest.h>

#include <tmc_manipulation_tests/configs.hpp>

using tmc_manipulation_types::ObjectParameterSeq;
using tmc_manipulation_types::OuterObjectParameters;
using tmc_manipulation_types::OuterObjectParametersSeq;
using tmc_manipulation_types::Cuboid;
using tmc_manipulation_types::CuboidSeq;
using tmc_manipulation_types::AABB;
using tmc_manipulation_types::JointState;

using ::testing::TestWithParam;
using ::testing::Values;

namespace {
const char* const kNonExistFile = "hogehoge.xml";
const char* const kIncorrectRobotModel = "gtest/incorrect_robot_model.xml";

const char* const kWall = "Wall";
const char* const kBoxes = "ThreeBoxes";
const char* const kCuboid = "Cuboid";
const char* const kWristGroup = "ARM7";
const char* const kJoint = "CARM/HEAD/NECK_Y";

const char* const kWrist1 = "CARM/SHAPE_WRIST_P_1";
const char* const kWrist2 = "CARM/SHAPE_WRIST_Y";
const char* const kLinear1 = "CARM/SHAPE_LINEAR_1";
const char* const kBase1 = "BASE/SHAPE_BASE_1";
const char* const kBase2 = "BASE/SHAPE_BASE_2";
const char* const kBase4 = "BASE/SHAPE_BASE_4";


const char* const kOutputFile = "gtest.xml";

const double kExtendLength = 0.5;
const int32_t kTopN = 5;

uint32_t GetJointIndex(const JointState& named_angle,
                       const std::string& joint_name) {
  return std::distance(named_angle.name.begin(),
                       std::find(
                           named_angle.name.begin(), named_angle.name.end(),
                           joint_name));
}

}  // anonymous namespace

namespace tmc_robot_collision_detector {

class RobotCollisionDetectorTest
    : public ::testing::TestWithParam<std::string> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  RobotCollisionDetectorTest() {}
  virtual ~RobotCollisionDetectorTest() {}
 protected:
  void SetUp() override;
  RobotCollisionDetector::Ptr detector_;
  OuterObjectParameters wall_;
  OuterObjectParameters boxes_;
  CuboidSeq bounding_boxes_;
  std::string robot_description_;
  std::string collision_pair_list_;
  std::string wrist_shape_;
  std::string wrist_y_shape_;
  std::string linear_shape_;
  std::string base_shape_1_;
  std::string base_shape_2_;
};

void RobotCollisionDetectorTest::SetUp() {
  // Retrieve from the parameter server
  robot_description_ = tmc_manipulation_tests::hsra::GetUrdf();
  collision_pair_list_ = tmc_manipulation_tests::hsra::GetCollisionConfig();

  wrist_shape_ = std::string(kWrist1) + "/collision/0";
  wrist_y_shape_ = std::string(kWrist2) + "/collision/0";
  linear_shape_ = std::string(kLinear1) + "/collision/0";
  base_shape_1_ = std::string(kBase1) + "/collision/0";
  base_shape_2_ = std::string(kBase2) + "/collision/0";

  // Prepare information for the object to be created
  wall_.name.assign(kWall);
  wall_.origin_to_base.setIdentity();
  wall_.origin_to_base.translation() = Eigen::Vector3d(0.7, 0.0, 0.5);
  wall_.shape.resize(1);
  wall_.shape.at(0).type = tmc_manipulation_types::kBox;
  wall_.shape.at(0).dimensions.push_back(0.06);
  wall_.shape.at(0).dimensions.push_back(1.0);
  wall_.shape.at(0).dimensions.push_back(1.0);
  wall_.base_to_child.push_back(Eigen::Affine3d::Identity());

  boxes_.name.assign(kBoxes);
  boxes_.origin_to_base.setIdentity();
  boxes_.origin_to_base.translation() = Eigen::Vector3d(0.2, 0.5, 0.3);
  tmc_manipulation_types::Shape shape;
  shape.type = tmc_manipulation_types::kBox;
  shape.dimensions.assign(3, 0.05);
  boxes_.shape.assign(3, shape);
  boxes_.base_to_child.assign(3, Eigen::Affine3d::Identity());
  boxes_.base_to_child.at(1).translation() = Eigen::Vector3d(0.04, 0.0, 0.0);
  boxes_.base_to_child.at(2).translation() = Eigen::Vector3d(0.0, 0.0, 0.04);

  Cuboid bounding_box;
  bounding_box.box_extents = Eigen::Vector3d(0.05, 1.0, 1.0);
  bounding_box.box_transform.setIdentity();
  bounding_box.box_transform.translation() = Eigen::Vector3d(0.2, 0.0, 0.5);
  bounding_box.box_name.assign(kCuboid);
  bounding_box.box_name += '0';
  for (int32_t i = 0; i < 3; i++) {
    bounding_box.box_aabb(i, 1) =
        bounding_box.box_transform.translation().coeff(i) +
        bounding_box.box_extents.coeff(i) / 2.0;
    bounding_box.box_aabb(i, 0) =
        bounding_box.box_transform.translation().coeff(i) -
        bounding_box.box_extents.coeff(i) / 2.0;
  }
  bounding_boxes_.push_back(bounding_box);
  bounding_box.box_name.assign(kCuboid);
  bounding_box.box_name += '1';
  bounding_box.box_transform.translation() = Eigen::Vector3d(0.65, 0.0, 0.5);
  for (int32_t i = 0; i < 3; i++) {
    bounding_box.box_aabb(i, 1) =
        bounding_box.box_transform.translation().coeff(i) +
        bounding_box.box_extents.coeff(i) / 2.0;
    bounding_box.box_aabb(i, 0) =
        bounding_box.box_transform.translation().coeff(i) -
        bounding_box.box_extents.coeff(i) / 2.0;
  }
  bounding_boxes_.push_back(bounding_box);
  bounding_box.box_name.assign(kCuboid);
  bounding_box.box_name += '2';
  bounding_box.box_transform.translation() = Eigen::Vector3d(0.60, 0.0, 0.5);
  for (int32_t i = 0; i < 3; i++) {
    bounding_box.box_aabb(i, 1) =
        bounding_box.box_transform.translation().coeff(i) +
        bounding_box.box_extents.coeff(i) / 2.0;
    bounding_box.box_aabb(i, 0) =
        bounding_box.box_transform.translation().coeff(i) -
        bounding_box.box_extents.coeff(i) / 2.0;
  }
  bounding_boxes_.push_back(bounding_box);

  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, GetParam());
}

TEST_P(RobotCollisionDetectorTest, Constructor) {
  // Normal case: Load a regular file (ODE)
  EXPECT_NO_THROW(
      RobotCollisionDetector(robot_description_,
                             collision_pair_list_, GetParam()));

  // Abnormal case: Robot model file does not exist
  EXPECT_ANY_THROW(
      RobotCollisionDetector(kNonExistFile,
                             collision_pair_list_, GetParam()));

  // Abnormal case: Incorrect syntax in robot model file
  EXPECT_ANY_THROW(
      RobotCollisionDetector(kIncorrectRobotModel,
                             collision_pair_list_, GetParam()));

  // Abnormal case: Specify a non-existent physics engine
  EXPECT_ANY_THROW(
      RobotCollisionDetector(robot_description_,
                             collision_pair_list_, kNonExistFile));
}

TEST_P(RobotCollisionDetectorTest, ConstructorWithKinematicsModel) {
  // An external kinematics model is used, so the robot_model should change similarly with position/orientation set to the detector
  auto robot_model = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>(robot_description_);
  auto detector_with_model = std::make_shared<RobotCollisionDetector>(
      robot_model, robot_description_, collision_pair_list_, GetParam());

  detector_with_model->SetRobotTransform(Eigen::Translation3d(1.0, 2.0, 0.0) * Eigen::AngleAxisd());
  const auto pose_from_detector = detector_with_model->GetObjectTransform(kJoint);
  const auto pose_from_kinematics = robot_model->GetObjectTransform(kJoint);

  // Translation movement command given to the cart, so checking the position is sufficient
  EXPECT_DOUBLE_EQ(pose_from_detector.translation().x(), pose_from_kinematics.translation().x());
  EXPECT_DOUBLE_EQ(pose_from_detector.translation().y(), pose_from_kinematics.translation().y());
  EXPECT_DOUBLE_EQ(pose_from_detector.translation().z(), pose_from_kinematics.translation().z());
}

TEST_P(RobotCollisionDetectorTest, CreateOuterObject) {
  // Normal case: Normal case
  EXPECT_NO_THROW(detector_->CreateOuterObject(boxes_));
  EXPECT_NO_THROW(detector_->CreateOuterObject(wall_));
  EXPECT_EQ(2, detector_->GetAllOuterObjectParameters().size());

  // Abnormal case: Mismatch between number of shapes and poses for child objects
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, GetParam());
  OuterObjectParameters incorrect_parameters = boxes_;
  incorrect_parameters.base_to_child.resize(2);
  EXPECT_ANY_THROW(detector_->CreateOuterObject(incorrect_parameters));

  // Abnormal case: No name
  incorrect_parameters = boxes_;
  incorrect_parameters.name.clear();
  EXPECT_ANY_THROW(detector_->CreateOuterObject(incorrect_parameters));

  // Abnormal case: Trying to use an existing name
  EXPECT_NO_THROW(detector_->CreateOuterObject(boxes_));
  EXPECT_ANY_THROW(detector_->CreateOuterObject(boxes_));
}

TEST_P(RobotCollisionDetectorTest, CreateCuboids) {
  // Normal case: Enable interference checking
  EXPECT_NO_THROW(detector_->CreateCuboids(bounding_boxes_, true));
  EXPECT_EQ(3, detector_->GetAllOuterObjectParameters().size());

  // Normal case: Disable interference checking
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, GetParam());
  EXPECT_NO_THROW(detector_->CreateCuboids(bounding_boxes_, false));
  EXPECT_EQ(3, detector_->GetAllOuterObjectParameters().size());

  // Normal case: Specify a group to create a Cuboid
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, GetParam());
  EXPECT_NO_THROW(detector_->CreateCuboids(bounding_boxes_, false, "BODY"));
  EXPECT_EQ(3, detector_->GetAllOuterObjectParameters().size());

  // Abnormal case: Create the same Cuboid
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, GetParam());
  EXPECT_NO_THROW(detector_->CreateCuboids(bounding_boxes_, false));
  EXPECT_ANY_THROW(detector_->CreateCuboids(bounding_boxes_, false));

  // Abnormal case: Create a Cuboid with an empty name
  CuboidSeq no_name_box_;
  no_name_box_.resize(1, bounding_boxes_[0]);
  no_name_box_[0].box_name.clear();
  EXPECT_ANY_THROW(detector_->CreateCuboids(no_name_box_, false));

  // Abnormal case: Specify a non-existent group to create a Cuboid
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, GetParam());
  EXPECT_ANY_THROW(detector_->CreateCuboids(bounding_boxes_, false, "hoge"));
}

TEST_P(RobotCollisionDetectorTest, DestroyOuterObject) {
  // Normal case: Dispose of existing external objects
  detector_->CreateOuterObject(wall_);
  EXPECT_NO_THROW(detector_->DestroyOuterObject(kWall));
  EXPECT_EQ(0, detector_->GetAllOuterObjectParameters().size());

  // Normal case: Dispose of robot parts (do not dispose)
  detector_->CreateOuterObject(wall_);
  EXPECT_NO_THROW(detector_->DestroyOuterObject(wrist_shape_));
  EXPECT_EQ(1, detector_->GetAllOuterObjectParameters().size());

  // Normal case: Dispose of held objects (do not dispose)
  detector_->CreateOuterObject(boxes_);
  detector_->HoldObject(kBoxes, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);
  EXPECT_NO_THROW(detector_->DestroyOuterObject(kBoxes));
  EXPECT_EQ(2, detector_->GetAllOuterObjectParameters().size());

  // Normal case: Dispose of all external objects
  EXPECT_NO_THROW(detector_->DestroyAllOuterObject());
  EXPECT_EQ(0, detector_->GetAllOuterObjectParameters().size());

  // Normal case: Do not create external objects and dispose of all external objects
  EXPECT_NO_THROW(detector_->DestroyAllOuterObject());

  // Normal case: Dispose of Cuboid
  detector_->CreateCuboids(bounding_boxes_, false);
  EXPECT_NO_THROW(detector_->DestroyOuterObject(bounding_boxes_[0].box_name));
  EXPECT_EQ(2, detector_->GetAllOuterObjectParameters().size());

  // Abnormal case: Dispose of non-existent external objects
  EXPECT_ANY_THROW(detector_->DestroyOuterObject("hoge"));

  // Abnormal case: Dispose of an object that has already been disposed of
  detector_->CreateOuterObject(boxes_);
  EXPECT_NO_THROW(detector_->DestroyOuterObject(kBoxes));
  EXPECT_ANY_THROW(detector_->DestroyOuterObject(kBoxes));

  // Abnormal case: Dispose of child objects
  detector_->CreateOuterObject(boxes_);
  EXPECT_ANY_THROW(
      detector_->DestroyOuterObject(std::string(kBoxes) + std::string("#1")));
}

TEST_P(RobotCollisionDetectorTest, DestroyCuboids) {
  // Normal case: Dispose of Cuboid
  detector_->CreateOuterObject(wall_);
  detector_->CreateCuboids(bounding_boxes_, false);
  EXPECT_NO_THROW(detector_->DestroyCuboids());
  EXPECT_EQ(1, detector_->GetAllOuterObjectParameters().size());
}

TEST_P(RobotCollisionDetectorTest, GetObjectParameter) {
  // Normal case: Get parameters of created objects
  detector_->CreateOuterObject(wall_);
  OuterObjectParameters param = detector_->GetObjectParameter(kWall);
  EXPECT_EQ(std::string(kWall), param.name);

  // Normal case: Get parameters of robot parts
  param = detector_->GetObjectParameter(wrist_shape_);
  EXPECT_EQ(std::string(wrist_shape_), param.name);

  // Normal case: Get parameters of child objects
  detector_->CreateOuterObject(boxes_);
  param = detector_->GetObjectParameter(std::string(kBoxes) + "#1");
  EXPECT_EQ(std::string(kBoxes) + "#1", param.name);

  // Normal case: Get parameters of Cuboid
  detector_->CreateCuboids(bounding_boxes_, false);
  param = detector_->GetObjectParameter(std::string(kCuboid) + "1");
  EXPECT_EQ(std::string(kCuboid) + "1", param.name);

  // Abnormal case: Get parameters of non-existent objects
  EXPECT_ANY_THROW(detector_->GetObjectParameter("hoge"));
}

TEST_P(RobotCollisionDetectorTest, GetAllOuterObjectParameter) {
  // Normal case: When there are no external objects
  EXPECT_EQ(0, detector_->GetAllOuterObjectParameters().size());

  // Normal case: When there are two external objects
  detector_->CreateOuterObject(wall_);
  detector_->CreateOuterObject(boxes_);
  EXPECT_EQ(2, detector_->GetAllOuterObjectParameters().size());

  // Normal case: When there is a Cuboid
  detector_->CreateCuboids(bounding_boxes_, false);
  EXPECT_EQ(5, detector_->GetAllOuterObjectParameters().size());
}

TEST_P(RobotCollisionDetectorTest, EnableDisableObject) {
  // Normal case: Check operation of EnableObject for an external object
  detector_->CreateOuterObject(wall_);
  EXPECT_NO_THROW(detector_->EnableCollisionObject(kWall));

  // Normal case: Check operation of DisableObject for an external object
  EXPECT_NO_THROW(detector_->DisableCollisionObject(kWall));

  // Normal case: Check operation of EnableObject for robot parts
  EXPECT_NO_THROW(detector_->EnableCollisionObject(wrist_shape_));

  // Normal case: Check operation of DisableObject for robot parts
  EXPECT_NO_THROW(detector_->DisableCollisionObject(wrist_shape_));

  // Normal case: Check operation of EnableObject for child objects
  detector_->CreateOuterObject(boxes_);
  std::string child_object_name(std::string(kBoxes) + std::string("#1"));
  EXPECT_NO_THROW(detector_->EnableCollisionObject(child_object_name));

  // Normal case: Check operation of DisableObject for child objects
  EXPECT_NO_THROW(detector_->DisableCollisionObject(child_object_name));

  // Normal case: Check operation of EnableObject for Cuboid
  detector_->CreateCuboids(bounding_boxes_, false);
  std::string cuboid0_name(std::string(kCuboid) + "0");
  std::string cuboid1_name(std::string(kCuboid) + "1");
  EXPECT_NO_THROW(detector_->EnableCollisionObject(cuboid0_name));
  EXPECT_NE(detector_->GetObjectGroup(cuboid1_name),
            detector_->GetObjectGroup(cuboid0_name));

  // Normal case: Check operation of DisableObject for Cuboid
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, GetParam());
  detector_->CreateCuboids(bounding_boxes_, false);
  EXPECT_NO_THROW(detector_->DisableCollisionObject(cuboid0_name));
  EXPECT_NE(detector_->GetObjectGroup(cuboid1_name),
            detector_->GetObjectGroup(cuboid0_name));

  // Abnormal case: Check operation of EnableObject for a non-existent object
  EXPECT_ANY_THROW(detector_->EnableCollisionObject("hoge"));

  // Abnormal case: Check operation of DisableObject for a non-existent object
  EXPECT_ANY_THROW(detector_->DisableCollisionObject("hoge"));
}

TEST_P(RobotCollisionDetectorTest, OperateGroupProperty) {
  // Normal case: Check operation of GetObjectGroup for an external object
  detector_->CreateOuterObject(wall_);
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kWall),
            detector_->GetObjectGroup(kWall));

  // Normal case: Check operation of SetObjectGroup for an external object
  EXPECT_NO_THROW(detector_->SetObjectGroup(kWall, 4));
  EXPECT_EQ(4, detector_->GetObjectGroup(kWall));

  // Normal case: Check operation of SetObjectDefaultGroup for an external object
  EXPECT_NO_THROW(detector_->SetObjectDefaultGroup(kWall));
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kWall),
            detector_->GetObjectGroup(kWall));

  // Normal case: Check operation of GetObjectGroup for robot parts
  EXPECT_EQ(detector_->GetObjectDefaultGroup(wrist_shape_),
            detector_->GetObjectGroup(wrist_shape_));

  // Normal case: Check operation of SetObjectGroup for robot parts
  EXPECT_NO_THROW(detector_->SetObjectGroup(wrist_shape_, 4));
  EXPECT_EQ(4, detector_->GetObjectGroup(wrist_shape_));

  // Normal case: Check operation of SetObjectDefaultGroup for robot parts
  EXPECT_NO_THROW(detector_->SetObjectDefaultGroup(wrist_shape_));
  EXPECT_EQ(detector_->GetObjectDefaultGroup(wrist_shape_),
            detector_->GetObjectGroup(wrist_shape_));

  // Normal case: Check operation of GetObjectGroup for child objects
  detector_->CreateOuterObject(boxes_);
  std::string child_object_name(std::string(kBoxes) + std::string("#1"));
  EXPECT_EQ(detector_->GetObjectGroup(kBoxes),
            detector_->GetObjectGroup(child_object_name));

  // Normal case: Check operation of GetObjectDefaultGroup for child objects
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kBoxes),
            detector_->GetObjectDefaultGroup(child_object_name));

  // Normal case: Check operation of SetObjectGroup for child objects
  EXPECT_NO_THROW(detector_->SetObjectGroup(child_object_name, 4));
  EXPECT_EQ(4, detector_->GetObjectGroup(child_object_name));

  // Normal case: Check operation of SetObjectDefaultGroup for child objects
  EXPECT_NO_THROW(detector_->SetObjectDefaultGroup(child_object_name));
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kBoxes),
            detector_->GetObjectGroup(child_object_name));

  // Normal case: Check operation of GetObjectGroup for Cuboid
  detector_->DestroyAllOuterObject();
  detector_->CreateCuboids(bounding_boxes_, false);
  std::string cuboid_name(std::string(kCuboid) + "0");
  EXPECT_EQ(detector_->GetObjectDefaultGroup(cuboid_name),
            detector_->GetObjectGroup(cuboid_name));

  // Normal case: Check operation of SetObjectGroup for Cuboid
  EXPECT_NO_THROW(detector_->SetObjectGroup(cuboid_name, 4));
  EXPECT_EQ(4, detector_->GetObjectGroup(cuboid_name));

  // Normal case: Check operation of SetObjectDefaultGroup for Cuboid
  EXPECT_NO_THROW(detector_->SetObjectDefaultGroup(cuboid_name));
  EXPECT_EQ(detector_->GetObjectDefaultGroup(cuboid_name),
            detector_->GetObjectGroup(cuboid_name));

  // Abnormal case: Check operation of GetObjectGroup for a non-existent object
  EXPECT_ANY_THROW(detector_->GetObjectGroup("hoge"));

  // Abnormal case: Check operation of SetObjectGroup for a non-existent object
  EXPECT_ANY_THROW(detector_->SetObjectGroup("hoge", 4));

  // Abnormal case: Check operation of SetObjectDefaultGroup for a non-existent object
  EXPECT_ANY_THROW(detector_->SetObjectDefaultGroup("hoge"));
}

TEST_P(RobotCollisionDetectorTest, OperateFilterProperty) {
  // Normal case: Check operation of GetObjectFilter for an external object
  detector_->CreateOuterObject(wall_);
  EXPECT_EQ(detector_->GetObjectDefaultFilter(kWall),
            detector_->GetObjectFilter(kWall));

  // Normal case: Check operation of SetObjectFilter for an external object
  EXPECT_NO_THROW(detector_->SetObjectFilter(kWall, 4));
  EXPECT_EQ(4, detector_->GetObjectFilter(kWall));

  // Normal case: Check operation of SetObjectDefaultFilter for an external object
  EXPECT_NO_THROW(detector_->SetObjectDefaultFilter(kWall));
  EXPECT_EQ(detector_->GetObjectDefaultFilter(kWall),
            detector_->GetObjectFilter(kWall));

  // Normal case: Check operation of GetObjectFilter for robot parts
  EXPECT_EQ(detector_->GetObjectDefaultFilter(wrist_shape_),
            detector_->GetObjectFilter(wrist_shape_));

  // Normal case: Check operation of SetObjectFilter for robot parts
  EXPECT_NO_THROW(detector_->SetObjectFilter(wrist_shape_, 4));
  EXPECT_EQ(4, detector_->GetObjectFilter(wrist_shape_));

  // Normal case: Check operation of SetObjectDefaultFilter for robot parts
  EXPECT_NO_THROW(detector_->SetObjectDefaultFilter(wrist_shape_));
  EXPECT_EQ(detector_->GetObjectDefaultFilter(wrist_shape_),
            detector_->GetObjectFilter(wrist_shape_));

  // Normal case: Check operation of GetObjectFilter for child objects
  detector_->CreateOuterObject(boxes_);
  std::string child_object_name(std::string(kBoxes) + std::string("#1"));
  EXPECT_EQ(detector_->GetObjectFilter(kBoxes),
            detector_->GetObjectFilter(child_object_name));

  // Normal case: Check operation of GetObjectDefaultFilter for child objects
  EXPECT_EQ(detector_->GetObjectDefaultFilter(kBoxes),
            detector_->GetObjectDefaultFilter(child_object_name));

  // Normal case: Check operation of SetObjectFilter for child objects
  EXPECT_NO_THROW(detector_->SetObjectFilter(child_object_name, 4));
  EXPECT_EQ(4, detector_->GetObjectFilter(child_object_name));

  // Normal case: Check operation of SetObjectDefaultFilter for child objects
  EXPECT_NO_THROW(detector_->SetObjectDefaultFilter(child_object_name));
  EXPECT_EQ(detector_->GetObjectDefaultFilter(kBoxes),
            detector_->GetObjectFilter(child_object_name));

  // Normal case: Check operation of GetObjectFilter for Cuboid
  detector_->DestroyAllOuterObject();
  detector_->CreateCuboids(bounding_boxes_, false);
  std::string cuboid_name(std::string(kCuboid) + "0");
  EXPECT_EQ(detector_->GetObjectDefaultFilter(cuboid_name),
            detector_->GetObjectFilter(cuboid_name));

  // Normal case: Check operation of SetObjectFilter for Cuboid
  EXPECT_NO_THROW(detector_->SetObjectFilter(cuboid_name, 4));
  EXPECT_EQ(4, detector_->GetObjectFilter(cuboid_name));

  // Normal case: Check operation of SetObjectDefaultFilter for Cuboid
  EXPECT_NO_THROW(detector_->SetObjectDefaultFilter(cuboid_name));
  EXPECT_EQ(detector_->GetObjectDefaultFilter(cuboid_name),
            detector_->GetObjectFilter(cuboid_name));

  // Abnormal case: Check operation of GetObjectFilter for a non-existent object
  EXPECT_ANY_THROW(detector_->GetObjectFilter("hoge"));

  // Abnormal case: Check operation of SetObjectFilter for a non-existent object
  EXPECT_ANY_THROW(detector_->SetObjectFilter("hoge", 4));

  // Abnormal case: Check operation of SetObjectDefaultFilter for a non-existent object
  EXPECT_ANY_THROW(detector_->SetObjectDefaultFilter("hoge"));
}

TEST_P(RobotCollisionDetectorTest, DisableCollisionPairProperty) {
  // Arrange objects so that child objects collide with robot parts
  OuterObjectParameters boxes;
  boxes = boxes_;
  boxes.origin_to_base = detector_->GetObjectTransform("BASE/FRAME_BASE");
  detector_->CreateOuterObject(boxes);
  detector_->CreateOuterObject(wall_);
  detector_->DisableCollisionObject(kWall);

  tmc_manipulation_types::JointState named_angle;
  named_angle = detector_->GetRobotNamedAngle();
  uint32_t shoulder_p_index =
      GetJointIndex(named_angle, "CARM/SHOULDER_P");
  uint32_t elbow_p_index =
      GetJointIndex(named_angle, "CARM/ELBOW_P");

  named_angle.position(shoulder_p_index) = 1.0;
  named_angle.position(elbow_p_index) = 0.0;
  detector_->SetRobotNamedAngle(named_angle);
  detector_->DisableCollisionObject(linear_shape_);

  // Exclude from interference checking then perform interference checking,
  // Then enable interference checking and perform interference checking again
  // Normal case: Exclude from interference checking (robot parts/parent object)
  detector_->DisableCollisionCheckObjectToObject(base_shape_2_, kBoxes);
  std::vector<PairString> contact_list;
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(3, contact_list.size());
  detector_->EnableCollisionCheckObjectToObject(base_shape_2_, kBoxes);
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(6, contact_list.size());

  // Normal case: Exclude from interference checking (robot parts/child object)
  detector_->DisableCollisionCheckObjectToObject(
      base_shape_2_, std::string(kBoxes) + "#0");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(5, contact_list.size());
  detector_->EnableCollisionCheckObjectToObject(
      base_shape_2_, std::string(kBoxes) + "#0");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(6, contact_list.size());

  // Normal case: Exclude from interference checking (robot parts/external object)
  detector_->DisableCollisionCheckObjectToGroup(base_shape_2_, "OUTER");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(3, contact_list.size());
  detector_->EnableCollisionCheckObjectToGroup(base_shape_2_, "OUTER");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(6, contact_list.size());

  // Normal case: Exclude from interference checking (part group/parent object)
  detector_->DisableCollisionCheckObjectToGroup(kBoxes, "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(0, contact_list.size());
  detector_->EnableCollisionCheckObjectToGroup(kBoxes, "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(6, contact_list.size());

  // Normal case: Exclude from interference checking (part group/child object)
  detector_->DisableCollisionCheckObjectToGroup(
      std::string(kBoxes) + "#0", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(4, contact_list.size());
  detector_->EnableCollisionCheckObjectToGroup(
      std::string(kBoxes) + "#0", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(6, contact_list.size());

  // Normal case: Exclude from interference checking (part group/external object)
  detector_->DisableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(0, contact_list.size());
  detector_->EnableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(6, contact_list.size());

  // Normal case: Verify if group setting changes are reflected
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, GetParam());
  detector_->DisableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->CreateOuterObject(boxes);
  detector_->SetRobotNamedAngle(named_angle);
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(0, contact_list.size());
  detector_->EnableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(6, contact_list.size());

  // Abnormal case: Specify a non-existent object name (object object)
  EXPECT_ANY_THROW(detector_->DisableCollisionCheckObjectToObject(
      base_shape_2_, "hoge"));

  // Abnormal case: Specify a non-existent object name (object group)
  EXPECT_ANY_THROW(detector_->DisableCollisionCheckObjectToGroup(
      "hoge", "BODY"));

  // Abnormal case: Specify a non-existent group name (object group)
  EXPECT_ANY_THROW(detector_->DisableCollisionCheckObjectToGroup(
      base_shape_2_, "hoge"));

  // Abnormal case: Specify a non-existent group name (group group)
  EXPECT_ANY_THROW(detector_->DisableCollisionCheckGroupToGroup(
      "BODY", "hoge"));
}

TEST_P(RobotCollisionDetectorTest, EnableCollisionPairProperty) {
  // Arrange objects so that child objects collide with robot parts
  OuterObjectParameters boxes;
  boxes = boxes_;
  boxes.origin_to_base = detector_->GetObjectTransform("BASE/FRAME_BASE");
  detector_->CreateOuterObject(boxes);
  detector_->CreateOuterObject(wall_);
  detector_->DisableCollisionObject(kWall);

  tmc_manipulation_types::JointState named_angle;
  named_angle = detector_->GetRobotNamedAngle();
  uint32_t shoulder_p_index =
      GetJointIndex(named_angle, "CARM/SHOULDER_P");
  uint32_t elbow_p_index =
      GetJointIndex(named_angle, "CARM/ELBOW_P");

  named_angle.position(shoulder_p_index) = 1.0;
  named_angle.position(elbow_p_index) = 0.0;
  detector_->SetRobotNamedAngle(named_angle);
  detector_->DisableCollisionObject(linear_shape_);

  // Add to interference checking and perform interference checking,
  // Then disable interference checking and perform interference checking again
  // Normal case: Add to interference checking after grasping (robot parts/parent object)
  detector_->HoldObject(kBoxes, "BASE/FRAME_BASE",
                        Eigen::Affine3d::Identity(), "BODY");
  detector_->EnableCollisionCheckObjectToObject(base_shape_2_, kBoxes);
  std::vector<PairString> contact_list;
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(3, contact_list.size());
  detector_->DisableCollisionCheckObjectToObject(base_shape_2_, kBoxes);
  detector_->CheckCollision(false, contact_list);
  EXPECT_TRUE(contact_list.empty());

  // Normal case: Add to interference checking (robot parts/child object)
  detector_->EnableCollisionCheckObjectToObject(
      base_shape_2_, std::string(kBoxes) + "#0");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(1, contact_list.size());
  detector_->DisableCollisionCheckObjectToObject(
      base_shape_2_, std::string(kBoxes) + "#0");
  detector_->CheckCollision(false, contact_list);
  EXPECT_TRUE(contact_list.empty());

  // Normal case: Add to interference checking (robot parts/external object)
  detector_->EnableCollisionCheckObjectToGroup(base_shape_2_, "OUTER");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(0, contact_list.size());
  detector_->DisableCollisionCheckObjectToGroup(base_shape_2_, "OUTER");
  detector_->CheckCollision(false, contact_list);
  EXPECT_TRUE(contact_list.empty());

  // Normal case: Add to interference checking (part group/parent object)
  detector_->EnableCollisionCheckObjectToGroup(kBoxes, "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(9, contact_list.size());
  detector_->DisableCollisionCheckObjectToGroup(kBoxes, "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_TRUE(contact_list.empty());

  // Normal case: Add to interference checking (part group/child object)
  detector_->EnableCollisionCheckObjectToGroup(
      std::string(kBoxes) + "#0", "BODY");
  detector_->CheckCollision(false, contact_list);
  std::cerr << std::endl;
  EXPECT_EQ(2, contact_list.size());
  detector_->DisableCollisionCheckObjectToGroup(
      std::string(kBoxes) + "#0", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_TRUE(contact_list.empty());

  // Normal case: Add to interference checking (part group/external object)
  detector_->EnableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(0, contact_list.size());
  detector_->DisableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->CheckCollision(false, contact_list);
  EXPECT_TRUE(contact_list.empty());

  // Normal case: Verify if group setting changes are reflected
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, GetParam());
  detector_->DisableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->EnableCollisionCheckGroupToGroup("OUTER", "BODY");
  detector_->CreateOuterObject(boxes);
  detector_->SetRobotNamedAngle(named_angle);
  detector_->CheckCollision(false, contact_list);
  EXPECT_EQ(6, contact_list.size());

  // Abnormal case: Specify a non-existent object name (object object)
  EXPECT_ANY_THROW(detector_->EnableCollisionCheckObjectToObject(
      base_shape_2_, "hoge"));

  // Abnormal case: Specify a non-existent object name (object group)
  EXPECT_ANY_THROW(detector_->EnableCollisionCheckObjectToGroup(
      "hoge", "BODY"));

  // Abnormal case: Specify a non-existent group name (object group)
  EXPECT_ANY_THROW(detector_->EnableCollisionCheckObjectToGroup(
      base_shape_2_, "hoge"));

  // Abnormal case: Specify a non-existent group name (group group)
  EXPECT_ANY_THROW(detector_->EnableCollisionCheckGroupToGroup(
      "BODY", "hoge"));
}

TEST_P(RobotCollisionDetectorTest, GetObjectTransform) {
  // Normal case: Get the posture of robot parts (shape)
  EXPECT_NO_THROW(detector_->GetObjectTransform(wrist_shape_));

  // Normal case: Get the posture of robot parts (joint)
  EXPECT_NO_THROW(detector_->GetObjectTransform(kJoint));

  // Normal case: Get the posture of external objects
  detector_->CreateOuterObject(wall_);
  EXPECT_NO_THROW(detector_->GetObjectTransform(kWall));

  // Normal case: Get the posture of held objects
  detector_->HoldObject(kWall, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);
  EXPECT_NO_THROW(detector_->GetObjectTransform(kWall));

  // Normal case: Get the posture of Cuboid
  detector_->CreateCuboids(bounding_boxes_, false);
  EXPECT_NO_THROW(detector_->GetObjectTransform(std::string(kCuboid) + "0"));

  // Abnormal case: Get the posture of non-existent objects
  EXPECT_ANY_THROW(detector_->GetObjectTransform("hoge"));
}

TEST_P(RobotCollisionDetectorTest, SetObjectTransform) {
  // Normal case: Set the posture of external objects
  detector_->CreateOuterObject(wall_);
  EXPECT_NO_THROW(
      detector_->SetObjectTransform(kWall, Eigen::Affine3d::Identity()));

  // Normal case: Set the posture of robot parts (shape) (cannot set)
  EXPECT_NO_THROW(
      detector_->SetObjectTransform(wrist_shape_, Eigen::Affine3d::Identity()));

  // Normal case: Set the posture of held objects (cannot set)
  detector_->HoldObject(kWall, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);
  EXPECT_NO_THROW(
      detector_->SetObjectTransform(kWall, Eigen::Affine3d::Identity()));

  // Normal case: Change the posture of Cuboid
  detector_->CreateCuboids(bounding_boxes_, false);
  std::string cuboid_name(std::string(kCuboid) + "0");
  EXPECT_NO_THROW(
      detector_->SetObjectTransform(cuboid_name, Eigen::Affine3d::Identity()));
  EXPECT_FALSE(detector_->GetObjectParameter(cuboid_name).cuboid);

  // Abnormal case: Set the posture of non-existent objects
  EXPECT_ANY_THROW(
      detector_->SetObjectTransform("hoge", Eigen::Affine3d::Identity()));
}

TEST_P(RobotCollisionDetectorTest, HoldObject) {
  // Normal case: A child object grasps one object
  detector_->CreateOuterObject(wall_);
  EXPECT_NO_THROW(detector_->HoldObject(kWall, wrist_shape_,
                                        Eigen::Affine3d::Identity(),
                                        kWristGroup));
  EXPECT_EQ(detector_->GetObjectFilter(wrist_shape_),
            detector_->GetObjectFilter(kWall));
  Eigen::Vector3d diff(
      detector_->GetObjectTransform(kWall).translation() -
      detector_->GetObjectTransform(wrist_shape_).translation());
  EXPECT_TRUE(diff.isZero());

  // Normal case: A child object grasps multiple objects
  detector_->CreateOuterObject(boxes_);
  EXPECT_NO_THROW(detector_->HoldObject(kBoxes, wrist_shape_,
                                        Eigen::Affine3d::Identity(),
                                        kWristGroup));
  EXPECT_EQ(detector_->GetObjectFilter(wrist_shape_),
            detector_->GetObjectFilter(kBoxes));
  diff = detector_->GetObjectTransform(kBoxes).translation() -
      detector_->GetObjectTransform(wrist_shape_).translation();
  EXPECT_TRUE(diff.isZero());

  // Normal case: Grasp an already held object (cannot grasp again)
  EXPECT_NO_THROW(detector_->HoldObject(kBoxes, wrist_shape_,
                                        Eigen::Affine3d::Identity(),
                                        kWristGroup));

  // Normal case: Grasp a Cuboid
  detector_->CreateCuboids(bounding_boxes_, false);
  EXPECT_NO_THROW(detector_->HoldObject(std::string(kCuboid) + "0", wrist_shape_,
                                        Eigen::Affine3d::Identity(),
                                        kWristGroup));

  // Normal case: Grasp robot parts (cannot grasp)
  EXPECT_NO_THROW(detector_->HoldObject(wrist_shape_, wrist_shape_,
                                        Eigen::Affine3d::Identity(),
                                        kWristGroup));

  // Normal case: Deduce group from frame_name and grasp
  detector_->ReleaseObject(kBoxes);
  EXPECT_NE(detector_->GetObjectFilter(wrist_shape_), detector_->GetObjectFilter(kBoxes));
  EXPECT_NO_THROW(detector_->HoldObject(kBoxes, "CARM/HAND/_root_", Eigen::Affine3d::Identity()));
  const auto config = std::make_shared<CollisionDetectorConfig>(collision_pair_list_);
  EXPECT_EQ(config->GetFilterBitByGroupName("HAND_GRASPED"), detector_->GetObjectFilter(kBoxes));

  // Abnormal case: Grasp non-existent objects
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, GetParam());
  detector_->CreateOuterObject(boxes_);
  EXPECT_ANY_THROW(detector_->HoldObject("hoge", wrist_shape_,
                                         Eigen::Affine3d::Identity(),
                                         kWristGroup));

  // Abnormal case: Grasp with a non-existent frame
  EXPECT_ANY_THROW(detector_->HoldObject(kBoxes, "hoge",
                                         Eigen::Affine3d::Identity(),
                                         kWristGroup));

  // Abnormal case: Grasp with a non-existent group
  EXPECT_ANY_THROW(detector_->HoldObject(kBoxes, wrist_shape_,
                                         Eigen::Affine3d::Identity(),
                                         "hoge"));
}

TEST_P(RobotCollisionDetectorTest, ReleaseObject) {
  // Normal case: A child object releases one object
  detector_->CreateOuterObject(wall_);
  detector_->HoldObject(kWall, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);
  EXPECT_NO_THROW(detector_->ReleaseObject(kWall));
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kWall),
            detector_->GetObjectGroup(kWall));

  // Normal case: A child object releases multiple objects
  detector_->CreateOuterObject(boxes_);
  detector_->HoldObject(kBoxes, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);
  EXPECT_NO_THROW(detector_->ReleaseObject(kBoxes));
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kBoxes),
            detector_->GetObjectGroup(kBoxes));

  // Normal case: Release all held objects
  detector_->HoldObject(kWall, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);
  detector_->HoldObject(kBoxes, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);
  EXPECT_NO_THROW(detector_->ReleaseAllObject());
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kWall),
            detector_->GetObjectGroup(kWall));
  EXPECT_EQ(detector_->GetObjectDefaultGroup(kBoxes),
            detector_->GetObjectGroup(kBoxes));

  // Normal case: Release a Cuboid
  detector_ = std::make_shared<RobotCollisionDetector>(robot_description_, collision_pair_list_, GetParam());
  detector_->CreateCuboids(bounding_boxes_, false);
  detector_->HoldObject(std::string(kCuboid) + "0", wrist_shape_,
                        Eigen::Affine3d::Identity(),
                        kWristGroup);
  EXPECT_NO_THROW(detector_->ReleaseObject(std::string(kCuboid) + "0"));
  EXPECT_NE(detector_->GetObjectGroup(std::string(kCuboid) + "0"),
            detector_->GetObjectGroup(std::string(kCuboid) + "1"));

  // Abnormal case: Release non-existent objects
  EXPECT_ANY_THROW(detector_->ReleaseObject("hoge"));

  // Abnormal case: Release objects not held
  detector_->CreateOuterObject(boxes_);
  EXPECT_ANY_THROW(detector_->ReleaseObject(kBoxes));
}

TEST_P(RobotCollisionDetectorTest, CollisionCheck) {
  // Arrange objects so that child objects collide with the robot
  OuterObjectParameters boxes;
  boxes = boxes_;
  boxes.origin_to_base = detector_->GetObjectTransform("BASE/FRAME_BASE");
  detector_->CreateOuterObject(boxes);

  tmc_manipulation_types::JointState named_angle;
  named_angle = detector_->GetRobotNamedAngle();
  uint32_t shoulder_p_index =
      GetJointIndex(named_angle, "CARM/SHOULDER_P");
  uint32_t elbow_p_index =
      GetJointIndex(named_angle, "CARM/ELBOW_P");

  named_angle.position(shoulder_p_index) = 1.0;
  named_angle.position(elbow_p_index) = 0.0;
  detector_->SetRobotNamedAngle(named_angle);

  // Normal case: Interference occurs
  EXPECT_TRUE(detector_->CheckCollision());

  // Normal case: Disable interference checking → No interference
  detector_->DisableCollisionObject(kBoxes);
  EXPECT_FALSE(detector_->CheckCollision());

  // Normal case: Enable after disabling → Interference occurs
  detector_->EnableCollisionObject(kBoxes);
  EXPECT_TRUE(detector_->CheckCollision());

  // Normal case: Grasping → No interference
  detector_->HoldObject(kBoxes, "BASE/FRAME_BASE",
                        Eigen::Affine3d::Identity(), "BODY");
  EXPECT_FALSE(detector_->CheckCollision());

  // Normal case: Release after grasping → Interference occurs
  detector_->ReleaseObject(kBoxes);
  EXPECT_TRUE(detector_->CheckCollision());

  // Normal case: Change filter setting to NOT interfere with BODY → No interference
  detector_->SetObjectGroup(
      kBoxes, detector_->GetObjectGroup(base_shape_1_));
  detector_->SetObjectFilter(
      kBoxes, detector_->GetObjectFilter(base_shape_1_));
  EXPECT_FALSE(detector_->CheckCollision());

  // Normal case: Revert group to original → Interference occurs
  detector_->SetObjectDefaultGroup(kBoxes);
  detector_->SetObjectDefaultFilter(kBoxes);
  EXPECT_TRUE(detector_->CheckCollision());

  // Normal case: Obtain names of interfering object pairs
  std::vector<PairString> contact_list;
  EXPECT_TRUE(detector_->CheckCollision(false, contact_list));
  ASSERT_FALSE(contact_list.empty());
  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  EXPECT_TRUE(detector_->CheckCollisionPair(
      contact_list[0].first, contact_list[0].second,
      point, normal));

  // Normal case: Obtain list of names of interfering object pairs
  EXPECT_TRUE(detector_->CheckCollision(true, contact_list));
  EXPECT_FALSE(contact_list.empty());

  // Normal case: Confirm default interference exclusion list is effective
  detector_->HoldObject(kBoxes, "BASE/FRAME_BASE",
                        Eigen::Affine3d::Identity(), "BODY");
  uint32_t wrist_p_index =
      GetJointIndex(named_angle, "CARM/WRIST_P");
  uint32_t neck_y_index =
      GetJointIndex(named_angle, "CARM/HEAD/NECK_Y");

  named_angle.position(wrist_p_index) = - M_PI / 2.0;
  named_angle.position(neck_y_index) = 69.0 / 180.0 * M_PI;
  detector_->SetRobotNamedAngle(named_angle);
  EXPECT_FALSE(detector_->CheckCollision());
}

TEST_P(RobotCollisionDetectorTest, GetAABB) {
  constexpr double kEpsilon = 1.0e-6;
  // Normal case: Obtain robot's AABB
  AABB default_aabb = detector_->GetRobotAABB();
  detector_->SetRobotTransform(
      Eigen::Translation3d(1.0, 0.0, 0.0) * Eigen::AngleAxisd::Identity());
  AABB moved_aabb = detector_->GetRobotAABB();
  EXPECT_NEAR(default_aabb(0, 0) + 1.0, moved_aabb(0, 0), kEpsilon);

  // Normal case: Obtain parent's AABB
  detector_->CreateCuboids(bounding_boxes_, true);
  default_aabb = detector_->GetObjectAABB(std::string(kCuboid) + "0");
  EXPECT_NEAR(0.175, default_aabb(0, 0), kEpsilon);
  EXPECT_NEAR(0.225, default_aabb(0, 1), kEpsilon);
  EXPECT_NEAR(-0.5, default_aabb(1, 0), kEpsilon);
  EXPECT_NEAR(0.5, default_aabb(1, 1), kEpsilon);
  EXPECT_NEAR(0.0, default_aabb(2, 0), kEpsilon);
  EXPECT_NEAR(1.0, default_aabb(2, 1), kEpsilon);

  // Normal case: Obtain child's AABB
  default_aabb = detector_->GetObjectAABB(std::string(kCuboid) + "0#0");
  EXPECT_NEAR(0.175, default_aabb(0, 0), kEpsilon);
  EXPECT_NEAR(0.225, default_aabb(0, 1), kEpsilon);
  EXPECT_NEAR(-0.5, default_aabb(1, 0), kEpsilon);
  EXPECT_NEAR(0.5, default_aabb(1, 1), kEpsilon);
  EXPECT_NEAR(0.0, default_aabb(2, 0), kEpsilon);
  EXPECT_NEAR(1.0, default_aabb(2, 1), kEpsilon);

  // Abnormal case: Obtain AABB of non-existent objects
  EXPECT_ANY_THROW(detector_->GetObjectAABB("hoge"));
}

TEST_P(RobotCollisionDetectorTest, RefleshOverlappedCuboids) {
  // Environment setup
  detector_->CreateCuboids(bounding_boxes_, false);
  tmc_manipulation_types::JointState named_angle;
  detector_->SetRobotTransform(
      Eigen::Translation3d(0.3, 0.0, 0.8) * Eigen::AngleAxisd::Identity());
  named_angle = detector_->GetRobotNamedAngle();
  uint32_t linear_index =
      GetJointIndex(named_angle, "CARM/LINEAR");
  uint32_t shoulder_p_index =
      GetJointIndex(named_angle, "CARM/SHOULDER_P");

  named_angle.position(linear_index) = 0.0;
  named_angle.position(shoulder_p_index) = 1.57;
  detector_->SetRobotNamedAngle(named_angle);

  // Normal case: Regular operation (aabb, group)
  EXPECT_NO_THROW(
      detector_->RefleshOverlappedCuboids(kOverlapAabb, kOverlapGroup));
  EXPECT_EQ(1, detector_->GetEnableCuboidsNum());

  // Normal case: Regular operation (2dmap, group)
  EXPECT_NO_THROW(
      detector_->RefleshOverlappedCuboids(kOverlap2DMap, kOverlapGroup));
  EXPECT_EQ(3, detector_->GetEnableCuboidsNum());

  // Normal case: Regular operation (aabb, robot)
  EXPECT_NO_THROW(
      detector_->RefleshOverlappedCuboids(kOverlapAabb, kOverlapRobot));
  EXPECT_EQ(3, detector_->GetEnableCuboidsNum());

  // Normal case: Regular operation (2dmap, robot)
  EXPECT_NO_THROW(
      detector_->RefleshOverlappedCuboids(kOverlap2DMap, kOverlapRobot));
  EXPECT_EQ(3, detector_->GetEnableCuboidsNum());
}

TEST_P(RobotCollisionDetectorTest, EnableDisableCuboids) {
  // Environment setup
  detector_->CreateCuboids(bounding_boxes_, false);
  detector_->SetRobotTransform(
      Eigen::Translation3d(0.2, 0.0, 0.0) * Eigen::AngleAxisd::Identity());
  tmc_manipulation_types::JointState named_angle;
  named_angle = detector_->GetRobotNamedAngle();
  uint32_t shoulder_p_index =
      GetJointIndex(named_angle, "CARM/SHOULDER_P");
  uint32_t elbow_p_index =
      GetJointIndex(named_angle, "CARM/ELBOW_P");

  named_angle.position(shoulder_p_index) = 1.0;
  named_angle.position(elbow_p_index) = 0.0;
  detector_->SetRobotNamedAngle(named_angle);

  // Normal case: Enable
  EXPECT_NO_THROW(detector_->EnableCuboids("CUBOID"));
  EXPECT_EQ(3, detector_->GetEnableCuboidsNum());
  EXPECT_TRUE(detector_->CheckCollision());

  // Normal case: Disable
  EXPECT_NO_THROW(detector_->DisableCuboids("CUBOID"));
  EXPECT_EQ(0, detector_->GetEnableCuboidsNum());
  EXPECT_FALSE(detector_->CheckCollision());

  // Normal case: Enable a group with existing groups, even though CUBOID is zero
  EXPECT_NO_THROW(detector_->EnableCuboids("BODY"));
  EXPECT_EQ(0, detector_->GetEnableCuboidsNum());
  EXPECT_FALSE(detector_->CheckCollision());

  // Normal case: Disable a group with existing groups, even though CUBOID is zero
  EXPECT_NO_THROW(detector_->EnableCuboids("CUBOID"));
  EXPECT_NO_THROW(detector_->DisableCuboids("BODY"));
  EXPECT_EQ(3, detector_->GetEnableCuboidsNum());
  EXPECT_TRUE(detector_->CheckCollision());

  // Abnormal case: Enable a non-existent group
  EXPECT_ANY_THROW(detector_->EnableCuboids("hoge"));

  // Abnormal case: Enable a non-existent group
  EXPECT_ANY_THROW(detector_->DisableCuboids("hoge"));
}

TEST_P(RobotCollisionDetectorTest, GetNameList) {
  // Environment construction
  detector_->CreateOuterObject(wall_);
  detector_->CreateOuterObject(boxes_);
  detector_->HoldObject(kWall, wrist_shape_,
                        Eigen::Affine3d::Identity(), kWristGroup);

  // Normal case: Obtain list of all object names in space
  std::vector<std::string> name_list;
  name_list = detector_->GetObjectNameList();
  EXPECT_EQ(29, name_list.size());

  // Normal case: Obtain list of object names included in the external object group
  name_list = detector_->GetObjectNameListByGroup("OUTER");
  EXPECT_EQ(1, name_list.size());

  // Normal case: Grasped objects are included in a group
  // Obtain list of object names
  name_list = detector_->GetObjectNameListByGroup(kWristGroup);
  EXPECT_EQ(3, name_list.size());

  // Normal case: Obtain list of object names included in the robot parts group
  name_list = detector_->GetObjectNameListByGroup("BODY");
  EXPECT_EQ(9, name_list.size());

  // Normal case: Obtain list of object names included in the CUBOID group
  detector_->CreateCuboids(bounding_boxes_, false);
  name_list = detector_->GetObjectNameListByGroup("CUBOID");
  EXPECT_EQ(3, name_list.size());

  // Normal case: Obtain list of object names included in the group where CUBOID has been added
  detector_->DestroyCuboids();
  detector_->CreateCuboids(bounding_boxes_, false, "BODY");
  name_list = detector_->GetObjectNameListByGroup("BODY");
  EXPECT_EQ(12, name_list.size());

  // Normal case: Obtain a group with empty objects
  name_list = detector_->GetObjectNameListByGroup("HANDHELD");
  EXPECT_TRUE(name_list.empty());

  // Normal case: Obtain list of group names
  name_list = detector_->GetGroupList();
  EXPECT_EQ(14, name_list.size());

  // Normal case: Obtain list of held object names
  name_list = detector_->GetHeldObjectList();
  EXPECT_EQ(1, name_list.size());

  // Abnormal case: Specify a non-existent group name
  EXPECT_ANY_THROW(detector_->GetObjectNameListByGroup("hoge"));
}

INSTANTIATE_TEST_CASE_P(MultiModelTypeTests,
                        RobotCollisionDetectorTest,
                        ::testing::Values("ODE", "fcl"));

}  // namespace tmc_robot_collision_detector

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
