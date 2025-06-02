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
/// @file robot_collision_detector_common.hpp
/// @brief Common settings for interference check using robot model
#ifndef TMC_ROBOT_COLLISION_DETECTOR_ROBOT_COLLISION_DETECTOR_COMMON_HPP_
#define TMC_ROBOT_COLLISION_DETECTOR_ROBOT_COLLISION_DETECTOR_COMMON_HPP_

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <tmc_collision_detector/collision_detector.hpp>
#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>

namespace tmc_robot_collision_detector {
/// Array of AABB
using AABBSeq = std::vector<tmc_manipulation_types::AABB, Eigen::aligned_allocator<tmc_manipulation_types::AABB>>;

/// Map of external objects
using OuterObjectMap =
    std::map<std::string, tmc_manipulation_types::OuterObjectParameters, std::less<std::string>,
             Eigen::aligned_allocator<std::pair<const std::string, tmc_manipulation_types::OuterObjectParameters>>>;

/// Structure holding Cuboid
struct BoundingBox {
  std::vector<std::string> group_name;
  std::vector<tmc_manipulation_types::CuboidSeq> boxes;
};

/// Criteria to determine overlap between Cuboid and robot
enum CuboidOverlapType {
  /// Determine overlap on XY plane
  kOverlap2DMap = 0,
  /// Determine overlap with AABB
  kOverlapAabb
};

/// How to handle robot's AABB
enum CuboidOverlapGroupType {
  /// Determine overlap for each group of robot's parts
  kOverlapGroup = 0,
  /// Determine overlap with entire robot's AABB
  kOverlapRobot
};

/// Search results of nearby objects
struct ClosestObject {
  /// Name of object to search
  std::string name;
  /// Results within robot
  tmc_collision_detector::ClosestResult inner_result;
  /// Results outside robot
  tmc_collision_detector::ClosestResult outer_result;
};

/// File type of robot model
enum ModelFileType {
  kTrml,  /// Model in trml (Toyota proprietary format) format
  kUrdf   /// Model in urdf (ROS standard) format
};

/// Information for collision position/orientation
struct CollisionFrameInfo {
  std::string parent_name;
  Eigen::Affine3d parent_to_child;
};
}  // namespace tmc_robot_collision_detector
#endif  // TMC_ROBOT_COLLISION_DETECTOR_ROBOT_COLLISION_DETECTOR_COMMON_HPP_
