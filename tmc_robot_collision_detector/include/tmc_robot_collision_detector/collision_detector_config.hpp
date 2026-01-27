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
/// @file collision_detector_config.hpp
/// @brief Holds the settings for interference check
#ifndef TMC_ROBOT_COLLISION_DETECTOR_COLLISION_DETECTOR_CONFIG_HPP_
#define TMC_ROBOT_COLLISION_DETECTOR_COLLISION_DETECTOR_CONFIG_HPP_

#include <stdint.h>

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>


namespace tmc_robot_collision_detector {

/// Exception when a non-existent object name is specified
class NonExistName : public std::domain_error {
 public:
  explicit NonExistName(const std::string& error) :
      std::domain_error("error: not exist name " + error) {}
};

/// Pair of strings, meaning varies by function
using PairString = std::pair<std::string, std::string>;

/// Group name of external objects
const char* const kOuterGroupName = "OUTER";

/// Default group name of collision map
const char* const kCuboidGroupName = "CUBOID";

/// Maximum number of groups
const uint32_t kMaxGroupNum = 15;

/// Settings of the interference checker
class CollisionDetectorConfig {
 public:
  using Ptr = std::shared_ptr<CollisionDetectorConfig>;

  /// @brief Constructor. Reads settings from a configuration file.
  /// @param [in] coldet_config_file Path to the configuration file
  /// @par Behavior:
  /// - Reads the configuration file and holds the settings as member variables.
  /// - Throws an exception if the configuration file cannot be read.
  /// - Throws an exception if the number of groups in the configuration file exceeds kMaxGroupNum.
  explicit CollisionDetectorConfig(const std::string& coldet_config_file);
  ~CollisionDetectorConfig() {}

  /// @brief Get the group bit of an object
  /// @param [in] object_name Name of the object to retrieve
  /// @return uint16_t Category of the object
  /// @par Behavior:
  /// - Returns the value of the group bit of the group to which the object belongs.
  /// - If a non-existent object name is given in the configuration file,
  ///   returns the value of the group bit of the external object.
  uint16_t GetGroupBitByObjectName(const std::string& object_name) const;

  /// @brief Get the filter bit of an object
  /// @param [in] object_name Name of the object to retrieve
  /// @return uint16_t Filter of the object
  /// @par Behavior:
  /// - Returns the value of the filter bit of the group to which the object belongs.
  /// - If a non-existent object name is given in the configuration file,
  ///   returns the value of the filter bit of the external object.
  uint16_t GetFilterBitByObjectName(const std::string& object_name) const;

  /// @brief Get the filter bit for searching neighboring objects within the robot
  /// @param [in] object_name Name of the object to retrieve
  /// @return uint16_t Filter of the object
  /// - Returns the value of the filter bit for searching neighboring objects
  ///   within the group to which the object belongs.
  /// - Throws an exception if a non-existent object name is given in the configuration file.
  uint16_t GetInnerFilterBit(const std::string& object_name) const;

  /// @brief Get the group bit of a group
  /// @param [in] group_name Name of the group to retrieve
  /// @return uint16_t Group bit of the group
  /// @par Behavior:
  /// - Returns the group bit according to the group name.
  /// - Throws an exception if a non-existent group name is given in the configuration file.
  uint16_t GetGroupBitByGroupName(const std::string& group_name) const;

  /// @brief Get the filter bit of a group
  /// @param [in] group_name Name of the group to retrieve
  /// @return uint16_t Filter of the group
  /// @par Behavior:
  /// - Returns the filter bit according to the group name.
  /// - Throws an exception if a non-existent group name is given in the configuration file.
  uint16_t GetFilterBitByGroupName(const std::string& group_name) const;

  /// @brief Get the group name from the object name
  /// @param [in] object_name Name of the object to retrieve
  /// @return std::string Group name to which the object belongs
  /// @par Behavior:
  /// - Throws the group name of the external object if a non-existent object name is given in the configuration file.
  ///
  std::string GetBelongedGroupName(const std::string& object_name) const;

  /// @brief Get the object names belonging to a group from the group name
  /// @param [in] group_name Name of the group to retrieve
  /// @return std::vector<std::string> Object names belonging to the group
  /// @par Behavior:
  /// - Returns an empty vector if a non-existent group name is given in the configuration file.
  std::vector<std::string> GetObjectListInGroup(
      const std::string& group_name) const;

  /// @brief Set the group and filter
  /// @param [in] group_name Name of the group to set
  /// @param [in] category Group bit of the group
  /// @param [in] filter Filter bit of the group
  /// @par Behavior:
  /// - Throws an exception if a non-existent group name is given.
  void SetConfig(const std::string& group_name,
                 uint16_t category, uint16_t filter);

  /// @brief Get the list of group names
  /// @return std::vector<std::string> List of group names
  std::vector<std::string> GetGroupNameList() const {return group_name_;}

  /// @brief Get the list of group names of robot parts
  /// @return std::vector<std::string> List of group names of robot parts
  std::vector<std::string> GetRobotPartsGroupNameList() const {
    return robot_parts_group_name_;
  }

  /// @brief Get the object pairs that do not perform interference check
  /// @return std::vector<PairString> Object pairs that do not perform interference check
  std::vector<PairString> GetDisableObjectPairList() const {
    return disable_collision_pair_list_;
  }

 private:
  /// @brief Set the category/filter to not perform interference check
  /// @param [in] groupA Name of the group to set
  /// @param [in] groupB Name of the group to set
  void SetNonContactPairToFilter_(const std::string& groupA,
                                  const std::string& groupB);

  /// @brief Set the category/filter to not perform interference check
  /// @param [in] groupA Name of the group to set
  /// @param [in] groupB Name of the group to set
  void SetNotCheckInnerPairToFilter_(const std::string& groupA,
                                     const std::string& groupB);

  /// List of group names
  std::vector<std::string> group_name_;
  /// Group names of robot parts
  std::vector<std::string> robot_parts_group_name_;
  /// Correspondence table of object and group names
  std::map<std::string, std::string> belonged_group_list_;
  /// Correspondence table of group names and objects
  std::multimap<std::string, std::string> group_member_list_;
  /// Correspondence table of group names and the group bits set by those groups
  std::map<std::string, uint16_t> group_bit_list_;
  /// Correspondence table of group names and the filter bits set by those groups (general use)
  std::map<std::string, uint16_t> filter_bit_list_;
  /// Correspondence table of group names and the filter bits set by those groups (internal use)
  std::map<std::string, uint16_t> inner_filter_bit_list_;
  /// List of object names that are excluded from interference check by default
  std::vector<PairString> disable_collision_pair_list_;
};
}  // namespace tmc_robot_collision_detector
#endif  // TMC_ROBOT_COLLISION_DETECTOR_COLLISION_DETECTOR_CONFIG_HPP_

