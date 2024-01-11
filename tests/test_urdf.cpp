// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
//
// SPDX-License-Identifier: GPL-3.0-only

#include <TinyURDFParser/TinyURDFParser.hpp>
#include <catch2/catch_test_macros.hpp>

#include <string>

#include "test_config.h"

static bool loadTestRobot(const std::string& urdf_file) {
    try {
        tup::TinyURDFParser parser = tup::TinyURDFParser::fromFile(urdf_file);
        return true;
    } catch (...) {
        return false;
    }
}

TEST_CASE("TinyURDFParser tests") {
    SECTION("Correct URDF File can be loaded") {
        REQUIRE(loadTestRobot(TEST_SOURCE_DIR + "/simple_robot.urdf") == true);
    }
    SECTION("Uncorrect URDF File can't be loaded") {
        REQUIRE(loadTestRobot(TEST_SOURCE_DIR + "/wrong_robot.urdf") == false);
    }
}
