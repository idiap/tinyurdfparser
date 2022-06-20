/**
    A lightweight URDF parser that convert an URDF file into a KDL Tree.

    Copyright (c) 2022 Idiap Research Institute, http://www.idiap.ch/
    Written by Jeremy Maceiras <jeremy.maceiras@idiap.ch>

    This file is part of TinyURDFParser.

    TinyURDFParser is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3 as
    published by the Free Software Foundation.

    TinyURDFParser is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with TinyURDFParser. If not, see <http://www.gnu.org/licenses/>.
*/

#include <Eigen/Dense>

#include <TinyURDFParser/KDLRobot.hpp>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    // Check the program call.
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <urdf_path>" << std::endl;
        return -1;
    }

    std::string urdf_file = std::string(argv[1]);

    // Create a KDL Robot object, with a kinematic chain starting at "panda_link0" and ending at "panda_tip"
    tup::sim::KDLRobot kdl_robot(urdf_file, "panda_link0", "panda_tip", Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));

    kdl_robot.printChain();

    // Print some values of interest
    std::cout << "------" << std::endl;
    std::cout << "Joint positions: " << kdl_robot.getJointsPos() << std::endl;
    std::cout << "------" << std::endl;
    std::cout << "Joint velocities: " << kdl_robot.getJointsVel() << std::endl;
    std::cout << "------" << std::endl;
    std::cout << "EE Pos: \n" << kdl_robot.getEEPosition() << std::endl;
    std::cout << "------" << std::endl;
    std::cout << "EE Vel: \n" << kdl_robot.getEEVelocity() << std::endl;
    std::cout << "------" << std::endl;
    std::cout << "EE Orn: \n" << kdl_robot.getEEOrnQuat() << std::endl;
    std::cout << "------" << std::endl;
    std::cout << "J: \n" << kdl_robot.J() << std::endl;

    return 0;
}