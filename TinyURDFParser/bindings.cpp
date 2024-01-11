// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
//
// SPDX-License-Identifier: GPL-3.0-only

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <TinyURDFParser/KDLRobot.hpp>
#include <eigen3/Eigen/Dense>

namespace py = pybind11;

PYBIND11_MODULE(PyTinyURDFParser, m) {
    py::class_<tup::sim::KDLRobot>(m, "KDLRobot")
        .def(py::init<const std::string&, const std::string&, const std::string&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&>(), py::arg("urdf"),
             py::arg("base_frame"), py::arg("tip_frame"), py::arg("q"), py::arg("dq"), py::arg("transform_rpy"), py::arg("transform_xyz"))
        .def("get_jacobian", &tup::sim::KDLRobot::J)
        .def("get_jacobian_derivative", &tup::sim::KDLRobot::Jp)
        .def("get_ee_pos", &tup::sim::KDLRobot::getEEPosition)
        .def("get_ee_vel", &tup::sim::KDLRobot::getEEVelocity)
        .def("get_ee_orn_quat", &tup::sim::KDLRobot::getEEOrnQuat)
        .def("get_q", &tup::sim::KDLRobot::getJointsPos)
        .def("get_dq", &tup::sim::KDLRobot::getJointsVel)
        .def("set_conf", &tup::sim::KDLRobot::setConfiguration, py::arg("q"), py::arg("dq"), py::arg("reset_time"))
        .def("send_acc", &tup::sim::KDLRobot::sendAcc, py::arg("dt"), py::arg("ddq"), py::arg("update_kin"))
        .def("send_vel", &tup::sim::KDLRobot::sendVel, py::arg("dt"), py::arg("dq"), py::arg("update_kin"));
}
