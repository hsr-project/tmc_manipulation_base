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
/// @file collision_detector_config-test.cpp
/// @brief Test class that holds the settings for interference check
#include <fstream>
#include <string>
#include <gtest/gtest.h>
#include "tmc_robot_collision_detector/collision_detector_config.hpp"

namespace {
std::string LoadFile(const std::string& file_path) {
  std::string xml_string;
  std::fstream xml_file(file_path, std::fstream::in);
  while (xml_file.good()) {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  return xml_string;
}
}  // namespace

namespace tmc_robot_collision_detector {

const int32_t kObjectNum = 7;
const char* kGroupName[kObjectNum] =
    {"GROUP1", "GROUP1", "GROUP1", "GROUP2",
    "GROUP2", "GROUP3", "OUTER"};
const char* kObjectName[kObjectNum] =
    {"OBJECT1", "OBJECT2", "OBJECT3", "OBJECT4",
    "OBJECT5", "OBJECT6", "OUTER1"};
const uint16_t kGroupAnswer[kObjectNum] = {1, 1, 1, 2, 2, 4, 8};
const uint16_t kFilterAnswer[kObjectNum] =
    {0xFFFC, 0xFFFC, 0xFFFC, 0xFFFC, 0xFFFC, 0xFFFB, 0xFFF7};
const uint16_t kInnerAnswer[kObjectNum] =
    {0xFFF8, 0xFFF8, 0xFFF8, 0xFFFC, 0xFFFC, 0xFFFA, 0xFFF7};

class CollisionDetectorConfigTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_test_ = LoadFile("gtest/test_config.xml");
    config_ = std::make_shared<CollisionDetectorConfig>(config_test_);
  }

  std::string config_test_;
  CollisionDetectorConfig::Ptr config_;
};

TEST_F(CollisionDetectorConfigTest, Constructor) {
  // Normal case: Load the correct configuration file
  EXPECT_EQ(4, config_->GetGroupNameList().size());

  // Abnormal case: Specify a configuration file with syntax errors
  EXPECT_ANY_THROW(std::make_shared<CollisionDetectorConfig>(LoadFile("gtest/incorrect_coldet_config.xml")));

  // Abnormal case: Load a configuration file where the number of groups exceeds the limit
  EXPECT_ANY_THROW(std::make_shared<CollisionDetectorConfig>(LoadFile("gtest/too_many_group_config.xml")));

  // Abnormal case: Set a non-existent group as an interference check exclusion pair
  // Load the configuration file
  EXPECT_ANY_THROW(std::make_shared<CollisionDetectorConfig>(LoadFile("gtest/non_exist_group_contact.xml")));

  // Abnormal case: Set a non-existent group as an interference check exclusion pair
  // Load the configuration file
  EXPECT_ANY_THROW(std::make_shared<CollisionDetectorConfig>(LoadFile("gtest/non_exist_group_inner.xml")));
}

TEST_F(CollisionDetectorConfigTest, GetBitByObject) {
  // Normal case: Specify object name to get group bit
  for (int32_t i = 0; i < kObjectNum; i++) {
    EXPECT_EQ(kGroupAnswer[i],
              config_->GetGroupBitByObjectName(kObjectName[i]));
  }

  // Normal case: Specify object name to get filter bit
  for (int32_t i = 0; i < kObjectNum; i++) {
    EXPECT_EQ(kFilterAnswer[i],
              config_->GetFilterBitByObjectName(kObjectName[i]));
  }

  // Normal case: Specify object name to get Inner bit
  for (int32_t i = 0; i < (kObjectNum-1); i++) {
    EXPECT_EQ(kInnerAnswer[i],
              config_->GetInnerFilterBit(kObjectName[i]));
  }

  // Abnormal case: Specify object name that is not a robot part to get Inner bit
  EXPECT_ANY_THROW(config_->GetInnerFilterBit(kObjectName[kObjectNum - 1]));
}

TEST_F(CollisionDetectorConfigTest, GetBitByGroup) {
  // Normal case: Specify group name to get group bit
  for (int32_t i = 0; i < kObjectNum; i++) {
    EXPECT_EQ(kGroupAnswer[i],
              config_->GetGroupBitByGroupName(kGroupName[i]));
  }

  // Normal case: Specify group name to get filter bit
  for (int32_t i = 0; i < kObjectNum; i++) {
    EXPECT_EQ(kFilterAnswer[i],
              config_->GetFilterBitByGroupName(kGroupName[i]));
  }

  // Abnormal case: Specify non-existent group name to get group bit
  EXPECT_ANY_THROW(config_->GetGroupBitByGroupName("hoge"));

  // Abnormal case: Specify non-existent group name to get filter bit
  EXPECT_ANY_THROW(config_->GetFilterBitByGroupName("hoge"));
}

TEST_F(CollisionDetectorConfigTest, SetConfig) {
  // Normal case: Set and changes are reflected
  uint16_t category = config_->GetGroupBitByGroupName(kGroupName[0]);
  uint16_t filter = config_->GetFilterBitByGroupName(kGroupName[0]);
  EXPECT_NO_THROW(config_->SetConfig(kGroupName[0], category - 1, filter - 1));
  EXPECT_EQ(category - 1, config_->GetGroupBitByGroupName(kGroupName[0]));
  EXPECT_EQ(filter - 1, config_->GetFilterBitByGroupName(kGroupName[0]));

  // Abnormal case: Specify non-existent group name to set
  EXPECT_ANY_THROW(config_->SetConfig("hoge", 1, 2));
}


TEST_F(CollisionDetectorConfigTest, GetName) {
  // Normal case: Get group name from object name
  for (int32_t i = 0; i < kObjectNum; i++) {
    EXPECT_EQ(kGroupName[i], config_->GetBelongedGroupName(kObjectName[i]));
  }

  // Normal case: When getting the group of a non-existent object, OUTER is returned
  EXPECT_EQ(std::string("OUTER"), config_->GetBelongedGroupName("hoge"));

  // Normal case: Get list of objects belonging to a group
  EXPECT_EQ(3, config_->GetObjectListInGroup(kGroupName[0]).size());
  EXPECT_EQ(2, config_->GetObjectListInGroup(kGroupName[3]).size());
  EXPECT_EQ(1, config_->GetObjectListInGroup(kGroupName[5]).size());
  EXPECT_EQ(0, config_->GetObjectListInGroup(kGroupName[6]).size());

  // Normal case: Get list of group names for robot parts
  EXPECT_EQ(3, config_->GetRobotPartsGroupNameList().size());

  // Normal case: Get list of object pairs excluded from interference check
  EXPECT_EQ(3, config_->GetDisableObjectPairList().size());

  // Abnormal case: Specify non-existent group name
  // Get list of objects belonging to a group
  EXPECT_ANY_THROW(config_->GetObjectListInGroup("hoge"));
}
}  // namespace tmc_robot_collision_detector
int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

