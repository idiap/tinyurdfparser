<!--
SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>

SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>

SPDX-License-Identifier: GPL-3.0-only
-->

# TinyURDFParser

TinyURDFParser is a lightweight URDF parser library, based on [TinyXML2](https://github.com/leethomason/tinyxml2), that converts an [URDF file] into a [KDL](https://www.orocos.org/kdl.html) object.

## Requirements

### Build requirements

* CMake 3.13 or newer.
* A C++ compiler, compatible with C++ 17 (tested with gcc 9.4.0)

### Libraries

* [TinyXML2](https://github.com/leethomason/tinyxml2) (zlib, added as submodule)
* [Catch2](https://github.com/catchorg/Catch2) (BSL-1.0, added as submodule)
* [PyBind11](https://github.com/pybind/pybind11) (custom license, added as submodule)
* [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) (MPL2)
* [orocos KDL](https://github.com/orocos/orocos_kinematics_dynamics) (LGPL-2.1)

## Installation

1. Clone this repository.
2. Install the dependencies.
  * For ``TinyXML2``, ``pybind11`` and ``Catch2`` (inside the project repository): ``git submodule update --init --recursive``
3. Perform a normal CMake based installation: ``mkdir build``, ``cd build``, ``cmake ..``, ``make``, ``make install``. CMakeLists comes with a list of option that you can toggle or not in function of your needs:
  * ``-DBUILD_TESTS={ON/OFF}`` to build the tests. To run them, in your build folder: ``make test``.
  * ``-DBUILD_EXAMPLES={ON/OFF}`` to build the C++ examples.
  * ``-DUSE_KDL={ON/OFF}`` to use KDL (needed for the python bindings).
  * ``-DBUILD_PYTHON_BINDINGS={ON/OFF}`` to build the python bindings.

## Usage

See ``examples`` folder for minimal use cases.
