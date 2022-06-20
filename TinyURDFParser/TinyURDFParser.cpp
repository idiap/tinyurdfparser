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

#include <tinyxml2/tinyxml2.h>
#include <map>

#include <TinyURDFParser/TinyURDFParser.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>

#include <iostream>

namespace tup {

TinyURDFParser::TinyURDFParser(const std::string& filename) {
    tinyxml2::XMLDocument urdf_xml;
    if (urdf_xml.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Unable to open urdf file");
    }

    // Build Translator for URDF Axis to KDL Axis
    urdf_to_kdl_type_["prismatic"] = KDL::Joint::JointType::TransAxis;
    urdf_to_kdl_type_["revolute"] = KDL::Joint::JointType::RotAxis;
    urdf_to_kdl_type_["fixed"] = KDL::Joint::JointType::None;
    urdf_to_kdl_type_["continuous"] = urdf_to_kdl_type_["revolute"];

    start_.name = "joint_root";

    init(urdf_xml);
}  // namespace tup

bool TinyURDFParser::setKinematicChain(const std::string& base, const std::string& tip) {
    return robot_tree_.getChain(base, tip, kinematic_chain_);
}

void TinyURDFParser::init(const tinyxml2::XMLDocument& urdf_xml) {
    const tinyxml2::XMLElement* root = urdf_xml.RootElement();

    if (root != NULL) {
        if (strcmp(root->Name(), "robot") == 0) {
            // Firstly parse all joints and links without connecting them

            std::map<std::string, Joint> joints;
            std::map<std::string, Segment> links;

            for (auto joint = root->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
                joints[std::string(joint->Attribute("name"))] = parseXMLJoint(joint);
            }

            for (auto link = root->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
                links[std::string(link->Attribute("name"))] = parseXMLSegment(link);
            }

            // Connect joints and links together
            if (joints.size() > 0 && links.size() > 0) {
                // Start by writing relation
                for (auto& [joint_name, joint] : joints) {
                    links[joint.parent_name].is_parent = true;
                    links[joint.parent_name].children.push_back(&joint);

                    links[joint.child_name].is_child = true;
                    links[joint.parent_name].parent = &joint;

                    joint.parent = &links[joint.parent_name];
                    joint.children.push_back(&links[joint.child_name]);
                }

                // Now go through all the segments to find the origin and the end
                for (auto& [link_name, link] : links) {
                    if (!link.is_parent && link.is_child) {
                        tips_.push_back(&link);
                    }
                    if (link.is_parent && !link.is_child) {
                        start_.children.push_back(&link);
                    }
                }
                buildKDLTree(&start_, "root");
            } else {
                std::runtime_error("[urdf_parsing] URDF file does not have <joint> or <link> tags");
            }

        } else {
            throw std::runtime_error("[urdf_parsing] Root element should be named <robot> not <" + std::string(root->Name()) + ">");
        }
    } else {
        throw std::runtime_error("[urdf_parsing] Given XML file does not contain a root element.");
    }
}

void TinyURDFParser::buildKDLTree(const KinematicElement* current, const std::string& hook) {
    std::string new_hook = hook;
    if (current->TAG == "JOINT") {
        const Joint* current_joint = static_cast<const Joint*>(current);

        std::string kdl_segment_name = current_joint->children.at(0)->name;
        KDL::Rotation kdl_transform_rpy = KDL::Rotation::RPY(current_joint->rpy(0), current_joint->rpy(1), current_joint->rpy(2));
        KDL::Vector kdl_transform_xyz(current_joint->xyz(0), current_joint->xyz(1), current_joint->xyz(2));
        KDL::Frame kdl_frame(kdl_transform_rpy, kdl_transform_xyz);

        KDL::Joint kdl_joint;

        if (hook != "root") {  // KC in KDL starts with a join, KC in URDF starts with a Link
            const Joint* previous_joint = static_cast<const Joint*>(current->parent->parent);

            std::string kdl_joint_name = previous_joint->name;

            if (previous_joint->type == "fixed") {  // Axis is fixed -> Less information to set
                KDL::Joint kdl_joint_tmp(kdl_joint_name, KDL::Joint::JointType::None);
                kdl_joint = kdl_joint_tmp;
            } else {  // Axis not fixed, need to specify origin and axis
                KDL::Vector kdl_origin = kdl_transform_xyz;
                KDL::Vector kdl_axis(previous_joint->axis(0), previous_joint->axis(1), previous_joint->axis(2));
                kdl_axis = kdl_transform_rpy * kdl_axis;
                KDL::Joint kdl_joint_tmp(kdl_joint_name, kdl_origin, kdl_axis, urdf_to_kdl_type_[previous_joint->type]);
                kdl_joint = kdl_joint_tmp;
            }
        }

        KDL::Segment kdl_segment(kdl_segment_name, kdl_joint, kdl_frame);

        bool connected = robot_tree_.addSegment(kdl_segment, hook);
        new_hook = kdl_segment_name;
    }

    for (auto child : current->children) {
        buildKDLTree(child, new_hook);
    }
}

TinyURDFParser::Joint TinyURDFParser::parseXMLJoint(const tinyxml2::XMLElement* joint_xml) {
    TinyURDFParser::Joint joint;

    // Firstly deal with mandatory attributes

    if (joint_xml->Attribute("name") != NULL)
        joint.name = std::string(joint_xml->Attribute("name"));
    else
        throw std::runtime_error("[urdf_parsing] Mandatory attribute \"name\" is missing for one of the joint");

    if (joint_xml->Attribute("type") != NULL) {
        joint.type = std::string(joint_xml->Attribute("type"));

        if (urdf_to_kdl_type_.find(joint.type) == urdf_to_kdl_type_.end()) {
            throw std::runtime_error("[urdf_parsing] Unknown joint type \"" + joint.type + "\"");
        }
    } else {
        throw std::runtime_error("[urdf_parsing] Mandatory attribute \"type\" is missing for joint " + joint.name);
    }

    if (joint_xml->FirstChildElement("parent")->Attribute("link") != NULL)
        joint.parent_name = std::string(joint_xml->FirstChildElement("parent")->Attribute("link"));
    else
        throw std::runtime_error("[urdf_parsing] Mandatory element <parent> is missing for joint " + joint.name);

    if (joint_xml->FirstChildElement("child")->Attribute("link") != NULL)
        joint.child_name = std::string(joint_xml->FirstChildElement("child")->Attribute("link"));
    else
        throw std::runtime_error("[urdf_parsing] Mandatory element <child> is missing for joint " + joint.name);

    // Now parse optional attribute

    if (joint_xml->FirstChildElement("origin") != NULL) {
        if (joint_xml->FirstChildElement("origin")->Attribute("rpy") != NULL) {
            std::string rpy_raw = std::string(joint_xml->FirstChildElement("origin")->Attribute("rpy"));
            joint.rpy = rawArrayToEigenVector(rpy_raw);
        }

        if (joint_xml->FirstChildElement("origin")->Attribute("xyz") != NULL) {
            std::string xyz_raw = std::string(joint_xml->FirstChildElement("origin")->Attribute("xyz"));
            joint.xyz = rawArrayToEigenVector(xyz_raw);
        }
    }

    if ((joint.type == "revolute" || joint.type == "continuous" || joint.type == "prismatic" || joint.type == "planar") && joint_xml->FirstChildElement("axis") != NULL) {
        if (joint_xml->FirstChildElement("axis")->Attribute("xyz") != NULL) {
            joint.axis_raw = std::string(joint_xml->FirstChildElement("axis")->Attribute("xyz"));
            joint.axis = rawArrayToEigenVector(joint.axis_raw);
        }
    }

    return joint;
}
TinyURDFParser::Segment TinyURDFParser::parseXMLSegment(const tinyxml2::XMLElement* segment_xml) {
    TinyURDFParser::Segment segment;
    segment.name = std::string(segment_xml->Attribute("name"));

    return segment;
}

Eigen::Vector3d TinyURDFParser::rawArrayToEigenVector(std::string raw_array) {
    const std::string delimiter = " ";
    Eigen::Vector3d array;

    size_t index = 0;
    size_t pos = 0;
    std::string token;
    while (true) {
        pos = raw_array.find(delimiter);

        if (index > 2) {
            throw std::runtime_error("[urdf_parsing] Expected a 3d array got more, check arrays format.");
        }

        token = raw_array.substr(0, pos);

        array(index) = std::stod(token);
        index++;

        if (pos == std::string::npos) {
            break;
        }

        raw_array.erase(0, pos + delimiter.length());
    }
    if (index != 3) {
        throw std::runtime_error("[urdf_parsing] Expected a 3d array got less, check arrays format.");
    }

    return array;
}

void TinyURDFParser::printKinematicChain(const TinyURDFParser::KinematicElement* elem, const int& level) {
    std::string spacing = "";
    for (int i = 0; i <= level; i++) {
        spacing += "  ";
    }

    std::cout << spacing << "TAG: " << elem->TAG << std::endl;

    if (elem->TAG == "JOINT") {
        const Joint* joint = static_cast<const Joint*>(elem);
        std::cout << spacing << "Name: " << joint->name << std::endl;
        std::cout << spacing << "Type: " << joint->type << std::endl;
        std::cout << spacing << "xyz: " << joint->xyz << std::endl;
        std::cout << spacing << "rpy: " << joint->rpy << std::endl;
        std::cout << spacing << "axis: " << joint->axis << std::endl;
    } else if (elem->TAG == "SEGMENT") {
        const Segment* link = static_cast<const Segment*>(elem);
        std::cout << spacing << "Name: " << link->name << std::endl;
    }

    int next_level = level + 1;
    for (auto child : elem->children) {
        printKinematicChain(child, next_level);
    }
}

}  // namespace tup