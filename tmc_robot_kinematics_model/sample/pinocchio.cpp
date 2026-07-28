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
#include <chrono>
#include <fstream>
#include <iostream>

#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>
#include <tmc_robot_kinematics_model/tarp3_wrapper.hpp>

using tmc_robot_kinematics_model::PinocchioWrapper;
using tmc_robot_kinematics_model::Tarp3Wrapper;
using tmc_robot_kinematics_model::IRobotKinematicsModel;

using tmc_manipulation_types::JointState;
using tmc_manipulation_types::NameSeq;

constexpr int EVAL_COUNT = 100000;

std::string load_hsrb_urdf() {
  // This is a quick script for operation check
  std::ifstream xml_file("/opt/ros/noetic/share/hsrb_description/robots/hsrb.urdf");
  std::string xml_string;
  while (xml_file.good()) {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }
  return xml_string;
}

void SetRobotTransform(IRobotKinematicsModel::Ptr robot) {
  robot->SetRobotTransform(Eigen::Translation3d(1.0, 2.0, 0.0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
}

void SetRobotTransform(IRobotKinematicsModel::Ptr robot, const std::string& name) {
  SetRobotTransform(robot);

  const auto transform = robot->GetRobotTransform();
  std::cout << name << "_robot_trasnform.R = " << std::endl
            << transform.linear() << std::endl
            << name << "_robot_trasnform.T = " << std::endl
            << transform.translation().transpose() << std::endl << std::endl;
}

void SetNamedAngle(IRobotKinematicsModel::Ptr robot) {
  JointState angle;
  angle.position.resize(5);
  angle.position << 0.19999988, -0.882339, -0.243443, 0.601559, 3.90538;
  angle.name = {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};
  robot->SetNamedAngle(angle);
}

void SetNamedAngle(IRobotKinematicsModel::Ptr robot, const std::string& name) {
  SetNamedAngle(robot);

  const auto angle = robot->GetNamedAngle();
  std::cout << name << "_joint_state:" << std::endl;
  for (int i = 0; i < angle.name.size(); ++i) {
    std::cout << "  " << angle.name[i] << ": " << angle.position[i] << std::endl;
  }
  std::cout << std::endl;
}

void GetObjectTransform(IRobotKinematicsModel::Ptr robot, const std::string& name, const std::string& frame_id) {
  const auto transform = robot->GetObjectTransform(frame_id);
  std::cout << name << "_" << frame_id << "_transform.R = " << std::endl
            << transform.linear() << std::endl
            << name << "_robot_trasnform.T = " << std::endl
            << transform.translation().transpose() << std::endl << std::endl;
}

void GetJacobian(IRobotKinematicsModel::Ptr robot, const std::string& name) {
  std::vector<std::string> joint_names = {
      "arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};
  std::cout << name << " jacobian of hand = " << std::endl
            << robot->GetJacobian("hand_palm_link", Eigen::Affine3d::Identity(), joint_names) << std::endl << std::endl;
}

void EvaluateFK(IRobotKinematicsModel::Ptr robot, const std::string& name) {
  const auto start = std::chrono::system_clock::now();
  for (int i = 0; i < EVAL_COUNT; ++i) {
    SetNamedAngle(robot);
    robot->GetObjectTransform("hand_palm_link");
  }
  const auto end = std::chrono::system_clock::now();

  std::cout << name << " fk: "
            << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / EVAL_COUNT
            << " [nsec/call]" << std::endl;
}

void EvaluateJacobian(IRobotKinematicsModel::Ptr robot, const std::string& name) {
  std::vector<std::string> joint_names = {
      "arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};

  const auto start = std::chrono::system_clock::now();
  for (int i = 0; i < EVAL_COUNT; ++i) {
    SetNamedAngle(robot);
    robot->GetJacobian("hand_palm_link", Eigen::Affine3d::Identity(), joint_names);
  }
  const auto end = std::chrono::system_clock::now();

  std::cout << name << " get jacobian: "
            << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / EVAL_COUNT
            << " [nsec/call]" << std::endl;
}

int main(int argc, char* argv[]) {
  const auto urdf_xml = load_hsrb_urdf();

  IRobotKinematicsModel::Ptr tarp3_robot(new Tarp3Wrapper(urdf_xml));
  IRobotKinematicsModel::Ptr pinocchio_robot(new PinocchioWrapper(urdf_xml));

  SetRobotTransform(tarp3_robot, "tarp3");
  SetRobotTransform(pinocchio_robot, "pinocchio");

  SetNamedAngle(tarp3_robot, "tarp3");
  SetNamedAngle(pinocchio_robot, "pinocchio");

  GetObjectTransform(tarp3_robot, "tarp3", "hand_palm_link");
  GetObjectTransform(pinocchio_robot, "pinocchio", "hand_palm_link");

  GetObjectTransform(tarp3_robot, "tarp3", "wrist_flex_joint");
  GetObjectTransform(pinocchio_robot, "pinocchio", "wrist_flex_joint");

  GetJacobian(tarp3_robot, "tarp3");
  GetJacobian(pinocchio_robot, "pinocchio");

  EvaluateFK(tarp3_robot, "tarp3");
  EvaluateFK(pinocchio_robot, "pinocchio");

  EvaluateJacobian(tarp3_robot, "tarp3");
  EvaluateJacobian(pinocchio_robot, "pinocchio");
  return EXIT_SUCCESS;
}
