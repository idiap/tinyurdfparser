# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

configure_file(test_config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/test_config.h)
add_executable(tests test_urdf.cpp test_config.h)
target_include_directories(tests PRIVATE test_config.h)
target_link_libraries(tests PRIVATE Catch2::Catch2WithMain TinyURDFParser)
add_test(NAME tests COMMAND tests)
