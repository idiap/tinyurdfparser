// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
//
// SPDX-License-Identifier: GPL-3.0-only

#include <iostream>

#include <TinyURDFParser/TinyURDFParser.hpp>
#include <kdl/chain.hpp>

int main(int argc, char* argv[]) {
    // Check the program call.
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <urdf_path>" << std::endl;
        return -1;
    }

    std::string urdf_file = std::string(argv[1]);

    tup::TinyURDFParser parser = tup::TinyURDFParser::fromFile(urdf_file);

    parser.setKinematicChain("panda_link0", "panda_tip");

    KDL::Chain chain = parser.getKinematicChain();

    std::cout << "The kinematics chain contains " << chain.getNrOfJoints() << " joints and " << chain.getNrOfSegments() << " segments" << std::endl;

    return 0;
}
