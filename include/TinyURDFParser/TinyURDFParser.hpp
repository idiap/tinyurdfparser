// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <map>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "TinyURDFParser/TinyURDFParser_config.h"  // cppcheck-suppress missingInclude

#ifdef USE_KDL
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#endif

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
    struct KinematicElement {
        std::string TAG = "TERMINATION";

        KinematicElement* parent = nullptr;
        std::vector<KinematicElement*> children;

        std::string name;
    };

    struct Geometry {
        std::string TAG = "UNKNOWN";

        Eigen::Vector3d xyz;
        Eigen::Vector3d rpy;
    };

    struct Joint : public KinematicElement {
        Joint() {
            TAG = "JOINT";
            axis_raw = "default";
            xyz = Eigen::Vector3d::Zero();
            rpy = Eigen::Vector3d::Zero();
            axis = Eigen::Vector3d(1, 0, 0);
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

        std::vector<Geometry*> primitives;
        bool is_parent;
        bool is_child;
    };

    struct Box : public Geometry {
        explicit Box(const Eigen::Vector3d& size) {
            TAG = "BOX";
            this->size = size;
        }

        Eigen::Vector3d size;
    };

    struct Cylinder : public Geometry {
        Cylinder(const double& radius, const double& length) {
            TAG = "CYLINDER";
            this->radius = radius;
            this->length = length;
        }

        double radius;
        double length;
    };

    struct Sphere : public Geometry {
        explicit Sphere(const double& radius) {
            TAG = "SPHERE";
            this->radius = radius;
        }

        double radius;
    };

    /**
     * @brief Construct a new TinyURDF Parser object
     *
     * @param filename path to the desired urdf file.
     */
    static TinyURDFParser fromFile(const std::string& filename);

    /**
     * @brief Construct a new TinyURDF Parser object
     *
     * @param filename URDF file's content.
     */
    explicit TinyURDFParser(const std::string& urdf);

    ~TinyURDFParser();

#ifdef USE_KDL

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
    KDL::Chain getKinematicChain() {
        return kinematic_chain_;
    }
#endif

    const std::map<std::string, Segment>& getLinks() {
        return links_;
    }
    const std::map<std::string, Joint>& getJoints() {
        return joints_;
    }

protected:
    Joint parseXMLJoint(const tinyxml2::XMLElement* joint_xml);
    Segment parseXMLSegment(tinyxml2::XMLElement* segment_xml);
    Eigen::Vector3d rawArrayToEigenVector(std::string raw_array);  // Copy is made on purpose

    Joint start_;
    std::vector<const KinematicElement*> tips_;

    void init(tinyxml2::XMLDocument& urdf_xml);

#ifdef USE_KDL
    KDL::Chain kinematic_chain_;
    KDL::Tree robot_tree_;

    std::map<std::string, KDL::Joint::JointType> urdf_to_kdl_type_;

    void buildKDLTree(const KinematicElement* current, const std::string& hook = "NONE");
#endif

    std::map<std::string, Joint> joints_;
    std::map<std::string, Segment> links_;
};
}  // namespace tup
