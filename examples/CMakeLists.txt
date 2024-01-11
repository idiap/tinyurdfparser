# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

add_executable(kdl_robot_example kdl_robot_example.cpp)
target_link_libraries(kdl_robot_example Eigen3::Eigen TinyURDFParser)

add_executable(urdf_parser_example urdf_parser_example.cpp)
target_link_libraries(urdf_parser_example ${orocos_kdl_LIBRARIES} TinyURDFParser)
