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

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>

namespace tinyxml2 {
class XMLElement;
class XMLDocument;
}  // namespace tinyxml2

namespace tup {

/**
 * @brief This class is a parser that converts an URDF file into a KDL Tree.
 *
 */
class TinyURDFParser {
public:
    /**
     * @brief Construct a new TinyURDF Parser object
     *
     * @param filename path to the desired urdf file.
     */
    TinyURDFParser(const std::string& filename);

    /**
     * @brief Create a new kinematic chain between <base> and <tip> links
     *
     * @param base name of the base link.
     * @param tip name of the tip link.
     * @return true managed to create the kinematic chain.
     * @return false  did not manage to create the kinematic chain.
     */
    bool setKinematicChain(const std::string& base, const std::string& tip);

    /**
     * @brief Get the Kinematic Chain object
     *
     * @return KDL::Chain
     */
    KDL::Chain getKinematicChain() { return kinematic_chain_; }

protected:
    struct KinematicElement {
        std::string TAG = "TERMINATION";

        KinematicElement* parent = nullptr;
        std::vector<KinematicElement*> children;

        std::string name;
    };

    struct Joint : public KinematicElement {
        Joint() {
            TAG = "JOINT";
            axis_raw = "default";
            xyz = Eigen::Vector3d::Zero();
            rpy = Eigen::Vector3d::Zero();
            axis << 1, 0, 0;
        }

        std::string parent_name;
        std::string child_name;
        std::string type;
        Eigen::Vector3d xyz;
        Eigen::Vector3d rpy;

        std::string axis_raw;
        Eigen::Vector3d axis;
    };

    struct Segment : public KinematicElement {
        Segment() {
            TAG = "SEGMENT";
            is_parent = false;
            is_child = false;
        }

        bool is_parent;
        bool is_child;
    };

    void printKinematicChain(const KinematicElement* elem, const int& level = 0);
    Joint parseXMLJoint(const tinyxml2::XMLElement* joint_xml);
    Segment parseXMLSegment(const tinyxml2::XMLElement* segment_xml);
    Eigen::Vector3d rawArrayToEigenVector(std::string raw_array);  // Copy is made on purpose

    void buildKDLTree(const KinematicElement* current, const std::string& hook = "NONE");

    Joint start_;
    std::vector<const KinematicElement*> tips_;

    void init(const tinyxml2::XMLDocument& urdf_xml);
    KDL::Chain kinematic_chain_;
    KDL::Tree robot_tree_;

    std::map<std::string, KDL::Joint::JointType> urdf_to_kdl_type_;
};
}  // namespace tup