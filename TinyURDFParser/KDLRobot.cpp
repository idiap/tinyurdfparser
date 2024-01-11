// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
//
// SPDX-License-Identifier: GPL-3.0-only

#include <TinyURDFParser/KDLRobot.hpp>
#include <TinyURDFParser/TinyURDFParser.hpp>
#include <iostream>

namespace tup {
namespace sim {

KDLRobot::KDLRobot(const std::string& urdf,
                   const std::string& base_frame,
                   const std::string& tip_frame,
                   const Eigen::VectorXd& q,
                   const Eigen::VectorXd& dq,
                   const Eigen::VectorXd& transform_rpy,
                   const Eigen::VectorXd& transform_xyz) {
    dof_ = q.rows();
    q_ = q;
    dq_ = dq;
    ddq_ = Eigen::VectorXd::Zero(dof_);

    x_ = Eigen::VectorXd::Zero(3);
    dx_ = x_;

    orn_quat_ = Eigen::VectorXd::Zero(4);
    w_ = Eigen::VectorXd::Zero(3);

    jac_ = Eigen::MatrixXd::Zero(6, dof_);
    djac_ = Eigen::MatrixXd::Zero(6, dof_);
    TinyURDFParser parser = TinyURDFParser::fromFile(urdf);
    KDL::Chain chain;
    if (parser.setKinematicChain(base_frame, tip_frame)) {
        chain = parser.getKinematicChain();
    } else {
        throw std::runtime_error("[KDLRobot] Unable to build kinematic chain from " + base_frame + " to " + tip_frame);
    }

    // Add virtual frame
    KDL::Rotation virtual_rot = KDL::Rotation::EulerZYX(transform_rpy(0), transform_rpy(1), transform_rpy(2));
    KDL::Vector virtual_pos(transform_xyz(0), transform_xyz(1), transform_xyz(2));
    KDL::Frame virtual_frame(virtual_rot, virtual_pos);
    KDL::Joint virtual_joint;
    KDL::Segment virtual_seg("robot_custom_tip", virtual_joint, virtual_frame);
    chain.addSegment(virtual_seg);

    initialize(chain);
    updateKinematics();
}

bool KDLRobot::initialize(const KDL::Chain& chain) {
    chain_ = chain;
    jacobian_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);
    fk_solver_pos_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);

    kdl_jacobian_ = KDL::Jacobian(chain.getNrOfJoints());
    kdl_positions_.resize(dof_);

    return true;
}

void KDLRobot::updateKinematics() {
    // Convert Eigen object to KDL one
    for (size_t i = 0; i < dof_; ++i) {
        kdl_positions_(i) = q_[i];
    }

    int error = 0;

    error += jacobian_solver_->JntToJac(kdl_positions_, kdl_jacobian_);
    error += fk_solver_pos_->JntToCart(kdl_positions_, kdl_pose_);

    if (error < 0) {
        throw std::runtime_error("[KDLRobot] Error while computing Jacobian and FK!");
    }

    for (size_t i = 0; i < 3; i++) {
        x_(i) = kdl_pose_.p[i];
    }

    orn_quat_ = Eigen::VectorXd::Zero(4);
    kdl_pose_.M.GetQuaternion(orn_quat_[1], orn_quat_[2], orn_quat_[3], orn_quat_[0]);

    jac_ = Eigen::MatrixXd::Zero(kdl_jacobian_.rows(), kdl_jacobian_.columns());
    for (size_t i = 0; i < kdl_jacobian_.rows(); i++) {
        for (size_t j = 0; j < kdl_jacobian_.columns(); j++) {
            jac_(i, j) = kdl_jacobian_(i, j);
        }
    }

    djac_ = getJacobianDerivative<7>(jac_, dq_);
    dx_ = jac_.topRows(3) * dq_;
    w_ = jac_.bottomRows(3) * dq_;
}

Eigen::MatrixXd KDLRobot::dQuatToDxJac(const Eigen::VectorXd& quat) {
    Eigen::MatrixXd J(3, 4);
    J << -quat(1), quat(0), -quat(3), quat(2), -quat(2), quat(3), quat(0), -quat(1), -quat(3), -quat(2), quat(1), quat(0);
    return J;
}

void KDLRobot::sendAcc(double dt, const Eigen::VectorXd& ddq, bool updateKin) {
    q_ += dt * dq_ + dt * dt / 2 * ddq;
    dq_ += dt * ddq;
    t_ += dt;
    if (updateKin)
        updateKinematics();
    ddq_ = ddq;
}

void KDLRobot::sendVel(double dt, const Eigen::VectorXd& dq, bool updateKin) {
    dq_ = dq;
    sendAcc(dt, Eigen::VectorXd::Zero(dq.rows()), updateKin);
}

Eigen::VectorXd KDLRobot::getEEAngVelQuat() {
    auto w = getEEAngVel();
    auto et = getEEOrnQuat();
    return .5 * dQuatToDxJac(et).transpose() * w;
}

void KDLRobot::setConfiguration(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, bool reset_time) {
    q_ = q;
    dq_ = dq;
    updateKinematics();
    if (reset_time) {
        t_ = 0;
    }
}

void KDLRobot::printChain() {
    std::cout << "==================================================" << std::endl;
    std::cout << " KDL Tree" << std::endl;

    for (int i = 0; i < chain_.getNrOfSegments(); i++) {
        std::cout << "----" << std::endl;

        std::cout << "Segment name: " << chain_.getSegment(i).getName() << std::endl;
        std::cout << "Segment xyz: " << chain_.getSegment(i).getFrameToTip().p.x() << " " << chain_.getSegment(i).getFrameToTip().p.y() << " " << chain_.getSegment(i).getFrameToTip().p.z()
                  << std::endl;

        double r, p, y;
        chain_.getSegment(i).getFrameToTip().M.GetRPY(r, p, y);
        std::cout << "Segment rpy: " << r << " " << p << " " << y << std::endl;

        std::cout << "Joint name: " << chain_.getSegment(i).getJoint().getName() << std::endl;
        std::cout << "Joint type: " << chain_.getSegment(i).getJoint().getTypeName() << std::endl;
        std::cout << "Joint axis: " << chain_.getSegment(i).getJoint().JointAxis().x() << " " << chain_.getSegment(i).getJoint().JointAxis().y() << " "
                  << chain_.getSegment(i).getJoint().JointAxis().z() << std::endl;
        std::cout << "Joint origin: " << chain_.getSegment(i).getJoint().JointOrigin().x() << " " << chain_.getSegment(i).getJoint().JointOrigin().y() << " "
                  << chain_.getSegment(i).getJoint().JointOrigin().z() << std::endl;
    }

    std::cout << "==================================================" << std::endl;
}

}  // namespace sim
}  // namespace tup
