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

    tup::TinyURDFParser parser(urdf_file);
    parser.setKinematicChain("panda_link0", "panda_tip");

    KDL::Chain chain = parser.getKinematicChain();

    std::cout << "The kinematics chain contains " << chain.getNrOfJoints() << " joints and " << chain.getNrOfSegments() << " segments" << std::endl;

    return 0;
}