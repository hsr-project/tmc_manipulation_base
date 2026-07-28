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
#include <iostream>

#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>
#include <tmc_robot_kinematics_model/tarp3_wrapper.hpp>
using tmc_robot_kinematics_model::Tarp3Wrapper;
using tmc_robot_kinematics_model::IRobotKinematicsModel;

using tmc_manipulation_types::JointState;
using tmc_manipulation_types::NameSeq;

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "usage: sample robot_trml_file" << std::endl;
  }
  try {
    IRobotKinematicsModel::Ptr robot(new Tarp3Wrapper(argv[1]));
    Eigen::Affine3d unit(Eigen::Affine3d::Identity());
    robot->SetRobotTransform(unit);
    JointState angle;
    angle.position.resize(7);
    angle.position(0) = 0.0;
    angle.position(1) = 0.0;
    angle.position(2) = 0.19999988;
    angle.position(3) = -0.882339;
    angle.position(4) = -0.243443;
    angle.position(5) = 0.601559;
    angle.position(6) = 3.90538;

    NameSeq use_name(7);
    use_name[0] = ("head_pan_joint");
    use_name[1] = ("head_tilt_joint");
    use_name[2] = ("arm_lift_joint");
    use_name[3] = ("arm_flex_joint");
    use_name[4] = ("arm_roll_joint");
    use_name[5] = ("wrist_flex_joint");
    use_name[6] = ("wrist_roll_joint");

    angle.name = use_name;
    robot->SetNamedAngle(angle);
    angle = robot->GetNamedAngle();

    Eigen::Affine3d hand = robot->GetObjectTransform("hand_palm_link");
    Eigen::Affine3d lift = robot->GetObjectTransform("arm_lift_link");

    Eigen::VectorXd min;
    Eigen::VectorXd max;
    robot->GetMinMax(use_name, min, max);
    std::cerr << "min = \n" << min << std::endl;
    std::cerr << "max = \n" << max << std::endl;

    std::cerr << "hand.R = \n" << hand.linear() << std::endl;
    std::cerr << "hand.t = \n" << hand.translation() << std::endl;

    std::cerr << "lift.R = \n" << lift.linear() << std::endl;
    std::cerr << "lift.t = \n" << lift.translation() << std::endl;

    std::cerr << "jac of hand = \n"
              << robot->GetJacobian("hand_palm_link", unit, use_name) << std::endl;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }
  return EXIT_SUCCESS;
}
