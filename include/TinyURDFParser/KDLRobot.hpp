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

#pragma once

#include <eigen3/Eigen/Dense>
#include <string>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <memory>

namespace tup {
namespace sim {

/**
 * @brief An helper class, abstracting a KDL Chain.
 *
 */
class KDLRobot {
public:
    /**
     * @brief Construct a new KDLRobot object
     *
     * @param urdf path to the urdf file
     * @param base_frame name of the base link of the kinematic chain
     * @param tip_frame name of the  tip link of the kinematic chain
     * @param q initial joint configuration
     * @param dq initial joint velocities
     * @param transform_rpy custom rotation to apply on the tip frame
     * @param transform_xyz custon translation to apply on the tip frame
     */
    KDLRobot(const std::string& urdf,
             const std::string& base_frame,
             const std::string& tip_frame,
             const Eigen::VectorXd& q,
             const Eigen::VectorXd& dq,
             const Eigen::VectorXd& transform_rpy = Eigen::VectorXd::Zero(3),
             const Eigen::VectorXd& transform_xyz = Eigen::VectorXd::Zero(3));

    virtual ~KDLRobot() {}

    /**
     * Forward kinematics function.
     * From the current configuration (q,dq), this method compute (x,dx,w,ornQuat,J,Jp)
     */
    void updateKinematics();

    /**
     * Return the translational Jacobian
     */
    Eigen::MatrixXd Jt() { return J().topRows(3); }

    /**
     * Return the rotational Jacobian
     */
    Eigen::MatrixXd Jr() { return J().bottomRows(3); }

    /**
     * Return the time derivative of the translational Jacobian
     */
    Eigen::MatrixXd Jtp() { return Jp().topRows(3); }

    /**
     * Return the time derivative of the rotational Jacobian
     */
    Eigen::MatrixXd Jrp() { return Jp().bottomRows(3); }

    /**
     * Return the full Jacobian
     */
    Eigen::MatrixXd J() { return jac_; }

    /**
     * Return the time derivative of the full Jacobian
     */
    Eigen::MatrixXd Jp() { return djac_; }

    /**
     * Return the quaternion derivative to angular velocities Jacobian
     */
    Eigen::MatrixXd dQuatToDxJac(const Eigen::VectorXd& quat);

    /**
     * Send a joint acceleration command to the robot
     */
    void sendAcc(double dt, const Eigen::VectorXd& ddq, bool updateKin = true);

    /**
     * Send a joint velocity command to the robot
     */
    void sendVel(double dt, const Eigen::VectorXd& dq, bool updateKin = true);

    /**
     * Return the end-effector position
     */
    Eigen::VectorXd getEEPosition() { return x_; }

    /**
     * Return the end-effector velocity
     */
    Eigen::VectorXd getEEVelocity() { return dx_; }

    /**
     * Return the end-effector angular velocity
     */
    Eigen::VectorXd getEEAngVel() { return w_; }
    Eigen::VectorXd getEEAngVelQuat();

    /**
     * Return the end-effector orientation
     */
    Eigen::VectorXd getEEOrnQuat() { return orn_quat_; }

    /**
     * Return the joint position of the robot
     */
    Eigen::VectorXd getJointsPos() { return q_; }

    /**
     * Return the joint velocities of the robot.
     */
    Eigen::VectorXd getJointsVel() { return dq_; }

    /**
     * Return the degree of freedom of the robot
     */
    int getDOF() { return dof_; }

    double getTime() { return t_; }
    void setTime(double time) { t_ = time; }

    /**
     * Update the current configuration of the robot
     */
    void setConfiguration(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, bool reset_time = true);

    void printChain();

protected:
    bool initialize(const KDL::Chain& chain);

    /**
     * Compute the time derivative of the Jacobian by firstly compute dJ/dq.
     * To get dJ/dt, we perfrorom a tensor multiplication: dJ/dt = dJ/dq * dq/dt
     *
     * Args:
     *      J Eigen::MatrixXd(6,7), the Jacobian.
     *      ddq Eigen::VectorXd(7), the joint angle acceleraitons.
     * Return:
     *      dJ/dt Eigen::MatrixXd(6,7)
     */
    template <size_t dof>
    inline Eigen::MatrixXd getJacobianDerivative(const Eigen::MatrixXd& jac, const Eigen::VectorXd& dq) {
        auto nb_rows = jac.rows();
        auto nb_cols = jac.cols();

        std::array<Eigen::MatrixXd, dof> J_grad;
        for (int i = 0; i < nb_cols; i++) {
            J_grad[i] = Eigen::MatrixXd::Zero(nb_rows, nb_cols);
        }

        for (int i = 0; i < nb_cols; i++) {
            for (int j = 0; j < nb_cols; j++) {
                auto J_i = jac.col(i);
                auto J_j = jac.col(j);

                if (j < i) {
                    J_grad[j](0, i) = J_j[4] * J_i[2] - J_j[5] * J_i[1];
                    J_grad[j](1, i) = J_j[5] * J_i[0] - J_j[3] * J_i[2];
                    J_grad[j](2, i) = J_j[3] * J_i[1] - J_j[4] * J_i[0];
                    J_grad[j](3, i) = J_j[4] * J_i[5] - J_j[5] * J_i[4];
                    J_grad[j](4, i) = J_j[5] * J_i[3] - J_j[3] * J_i[5];
                    J_grad[j](5, i) = J_j[3] * J_i[4] - J_j[4] * J_i[3];
                } else if (j > i) {
                    J_grad[j](0, i) = -J_j[1] * J_i[5] + J_j[2] * J_i[4];
                    J_grad[j](1, i) = -J_j[2] * J_i[3] + J_j[0] * J_i[5];
                    J_grad[j](2, i) = -J_j[0] * J_i[4] + J_j[1] * J_i[3];
                } else {
                    J_grad[j](0, i) = J_i[4] * J_i[2] - J_i[5] * J_i[1];
                    J_grad[j](1, i) = J_i[5] * J_i[0] - J_i[3] * J_i[2];
                    J_grad[j](2, i) = J_i[3] * J_i[1] - J_i[4] * J_i[0];
                }
            }
        }
        Eigen::MatrixXd dJ = Eigen::MatrixXd::Zero(nb_rows, nb_cols);

        for (int i = 0; i < nb_cols; i++) {
            Eigen::VectorXd Ji = Eigen::VectorXd::Zero(nb_rows);
            for (int j = 0; j < nb_cols; j++) {
                Ji += J_grad[j].col(i) * dq(j);
            }
            dJ.col(i) = Ji;
        }
        return dJ;
    }

    Eigen::VectorXd q_, dq_, ddq_;
    Eigen::VectorXd x_, dx_;
    Eigen::VectorXd orn_quat_, w_;
    Eigen::MatrixXd jac_, djac_;

    int dof_;
    double t_;

    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_pos_;

    KDL::JntArray kdl_positions_;
    KDL::Jacobian kdl_jacobian_;
    KDL::Frame kdl_pose_;
};
}  // namespace sim
}  // namespace tup
