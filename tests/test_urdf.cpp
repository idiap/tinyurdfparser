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

#include <TinyURDFParser/TinyURDFParser.hpp>
#include <catch2/catch_test_macros.hpp>

#include <string>

#include "test_config.h"

static bool loadTestRobot(const std::string& urdf_file) {
    try {
        tup::TinyURDFParser parser(urdf_file);
        return true;
    } catch (...) {
        return false;
    }
}

TEST_CASE("TinyURDFParser tests") {
    SECTION("Correct URDF File can be loaded") { REQUIRE(loadTestRobot(TEST_SOURCE_DIR + "/simple_robot.urdf") == true); }
    SECTION("Uncorrect URDF File can't be loaded") { REQUIRE(loadTestRobot(TEST_SOURCE_DIR + "/wrong_robot.urdf") == false); }
}