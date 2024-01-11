// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
//
// SPDX-License-Identifier: GPL-3.0-only

#include <tinyxml2/tinyxml2.h>
#include <map>

#include <TinyURDFParser/TinyURDFParser.hpp>

#ifdef USE_KDL
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>
#endif

#include <algorithm>
#include <fstream>
#include <iostream>

namespace tup {

TinyURDFParser TinyURDFParser::fromFile(const std::string& filename) {
    std::ifstream f(filename.c_str());
    std::stringstream buffer;

    buffer << f.rdbuf();

    TinyURDFParser parser(buffer.str());

    f.close();
    return parser;
}

TinyURDFParser::TinyURDFParser(const std::string& urdf) {
    tinyxml2::XMLDocument urdf_xml;
    if (urdf_xml.Parse(urdf.c_str()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Unable to open urdf file");
    }

#ifdef USE_KDL
    // Build Translator for URDF Axis to KDL Axis
    urdf_to_kdl_type_["prismatic"] = KDL::Joint::JointType::TransAxis;
    urdf_to_kdl_type_["revolute"] = KDL::Joint::JointType::RotAxis;
    urdf_to_kdl_type_["fixed"] = KDL::Joint::JointType::None;
    urdf_to_kdl_type_["continuous"] = urdf_to_kdl_type_["revolute"];
#endif

    start_.name = "joint_root";

    init(urdf_xml);
}

TinyURDFParser::~TinyURDFParser() {
    for (auto const& link : links_) {
        for (auto prim : link.second.primitives) {
            delete prim;
        }
    }
}

#ifdef USE_KDL
bool TinyURDFParser::setKinematicChain(const std::string& base, const std::string& tip) {
    return robot_tree_.getChain(base, tip, kinematic_chain_);
}
#endif

void TinyURDFParser::init(tinyxml2::XMLDocument& urdf_xml) {
    tinyxml2::XMLElement* root = urdf_xml.RootElement();

    if (root != NULL) {
        if (strcmp(root->Name(), "robot") == 0) {
            // Firstly parse all joints and links without connecting them

            for (auto joint = root->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
                joints_[std::string(joint->Attribute("name"))] = parseXMLJoint(joint);
            }

            for (tinyxml2::XMLElement* link = root->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
                links_[std::string(link->Attribute("name"))] = parseXMLSegment(link);
            }

            // Connect joints and links together
            if (joints_.size() > 0 && links_.size() > 0) {
                // Start by writing relation
                for (auto& joint : joints_) {
                    links_[joint.second.parent_name].is_parent = true;
                    links_[joint.second.parent_name].children.push_back(&joint.second);

                    links_[joint.second.child_name].is_child = true;
                    links_[joint.second.parent_name].parent = &joint.second;

                    joint.second.parent = &links_[joint.second.parent_name];
                    joint.second.children.push_back(&links_[joint.second.child_name]);
                }

                // Now go through all the segments to find the origin and the end
                for (auto& link : links_) {
                    if (!link.second.is_parent && link.second.is_child) {
                        tips_.push_back(&link.second);
                    }
                    if (link.second.is_parent && !link.second.is_child) {
                        start_.children.push_back(&link.second);
                    }
                }
#ifdef USE_KDL
                buildKDLTree(&start_, "root");
#endif
            } else {
                throw std::runtime_error("[urdf_parsing] URDF file does not have <joint> or <link> tags");
            }

        } else {
            throw std::runtime_error("[urdf_parsing] Root element should be named <robot> not <" + std::string(root->Name()) + ">");
        }
    } else {
        throw std::runtime_error("[urdf_parsing] Given XML file does not contain a root element.");
    }
}

#ifdef USE_KDL
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

        robot_tree_.addSegment(kdl_segment, hook);
        new_hook = kdl_segment_name;
    }

    for (auto child : current->children) {
        buildKDLTree(child, new_hook);
    }
}
#endif

TinyURDFParser::Joint TinyURDFParser::parseXMLJoint(const tinyxml2::XMLElement* joint_xml) {
    TinyURDFParser::Joint joint;

    // Firstly deal with mandatory attributes

    if (joint_xml->Attribute("name") != NULL)
        joint.name = std::string(joint_xml->Attribute("name"));
    else
        throw std::runtime_error("[urdf_parsing] Mandatory attribute \"name\" is missing for one of the joint");

    if (joint_xml->Attribute("type") != NULL) {
        joint.type = std::string(joint_xml->Attribute("type"));

#ifdef USE_KDL
        if (urdf_to_kdl_type_.find(joint.type) == urdf_to_kdl_type_.end()) {
            throw std::runtime_error("[urdf_parsing] Unknown joint type \"" + joint.type + "\"");
        }
#endif

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

TinyURDFParser::Segment TinyURDFParser::parseXMLSegment(tinyxml2::XMLElement* segment_xml) {
    TinyURDFParser::Segment segment;
    segment.name = std::string(segment_xml->Attribute("name"));

    for (tinyxml2::XMLElement* collision_elem = segment_xml->FirstChildElement("collision"); collision_elem != NULL; collision_elem = collision_elem->NextSiblingElement()) {
        Eigen::Vector3d rpy, xyz;

        if (collision_elem->FirstChildElement("origin") == NULL || collision_elem->FirstChildElement("geometry") == NULL) {
            continue;
        }

        if (collision_elem->FirstChildElement("origin")->Attribute("rpy") != NULL) {
            std::string rpy_raw = std::string(collision_elem->FirstChildElement("origin")->Attribute("rpy"));
            rpy = rawArrayToEigenVector(rpy_raw);
        }

        if (collision_elem->FirstChildElement("origin")->Attribute("xyz") != NULL) {
            std::string xyz_raw = std::string(collision_elem->FirstChildElement("origin")->Attribute("xyz"));
            xyz = rawArrayToEigenVector(xyz_raw);
        }

        if (collision_elem->FirstChildElement("geometry")->NoChildren()) {
            continue;
        }

        std::string type = collision_elem->FirstChildElement("geometry")->FirstChild()->Value();

        TinyURDFParser::Geometry* obj;

        if (type == "box") {
            obj = new TinyURDFParser::Box(rawArrayToEigenVector(std::string(collision_elem->FirstChildElement("geometry")->FirstChildElement()->Attribute("size"))));
        } else if (type == "cylinder") {
            double radius = std::stod(collision_elem->FirstChildElement("geometry")->FirstChildElement()->Attribute("radius"));
            double length = std::stod(collision_elem->FirstChildElement("geometry")->FirstChildElement()->Attribute("length"));
            obj = new TinyURDFParser::Cylinder(radius, length);
        } else if (type == "sphere") {
            double radius = std::stod(collision_elem->FirstChildElement("geometry")->FirstChildElement()->Attribute("radius"));
            obj = new TinyURDFParser::Sphere(radius);
        } else {
            continue;
        }

        obj->rpy = rpy;
        obj->xyz = xyz;
        segment.primitives.push_back(obj);
    }

    return segment;
}

Eigen::Vector3d TinyURDFParser::rawArrayToEigenVector(std::string raw_array) {
    // Trim from end
    raw_array.erase(std::find_if(raw_array.rbegin(), raw_array.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), raw_array.end());

    const std::string delimiter = " ";
    Eigen::Vector3d array;

    size_t index = 0;
    std::string token;
    while (true) {
        size_t pos = raw_array.find(delimiter);

        if (index > 2) {
            throw std::runtime_error("[urdf_parsing] Expected a 3d array got more, check arrays format.");
        }

        if (pos > 0) {
            token = raw_array.substr(0, pos);

            array[index] = std::stod(token);
            index++;
        }

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
}  // namespace tup
