#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <tinyxml.h>
#include <urdf_parser/urdf_parser.h>
#include <ignition/math.hh>
#include "freebuoyancy.h"
#include <ignition/math/Quaternion.hh>

using std::cout;
using std::endl;
using std::string;

namespace gazebo {

void FreeBuoyancyPlugin::ReadVector3(const std::string &_string, ignition::math::Vector3d &_vector) {
    std::stringstream ss(_string);
    double xyz[3];
    for (unsigned int i=0; i<3; ++i)
        ss >> xyz[i];
    _vector.Set(xyz[0], xyz[1], xyz[2]);
}

void FreeBuoyancyPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    cout << ("Loading freebuoyancy_gazebo plugin...\n");

    this->world_ = _model->GetWorld();

    // parse plugin options
    description_ = "robot_description";
    has_surface_ = false;
    surface_plane_.Set(0, 0, 1, 0); // default ocean surface plane is Z=0
    std::string fluid_topic = "current";

    if (_sdf->HasElement("descriptionParam"))  description_ = _sdf->Get<std::string>("descriptionParam");

    if (_sdf->HasElement("surface")) {
        has_surface_ = true;
        // get one surface point
        ignition::math::Vector3d surface_point;
        ReadVector3(_sdf->Get<std::string>("surface"), surface_point);
        // get gravity
        const ignition::math::Vector3d WORLD_GRAVITY = world_->Gravity().Normalize();
        // water surface is orthogonal to gravity
        surface_plane_.Set(WORLD_GRAVITY.X(), WORLD_GRAVITY.Y(), WORLD_GRAVITY.Z(), WORLD_GRAVITY.Dot(surface_point));
    }

    if (_sdf->HasElement("fluidTopic"))  fluid_topic = _sdf->Get<std::string>("fluidTopic");

    fluid_velocity_.Set(0, 0, 0);

    // Register plugin update
    update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&FreeBuoyancyPlugin::OnUpdate, this));

    // Clear existing links
    buoyant_links_.clear();
    parsed_models_.clear();

    cout << ("Loaded freebuoyancy_gazebo plugin.\n");
}

void FreeBuoyancyPlugin::OnUpdate() {

    // look for new world models
    unsigned int i;
    std::vector<model_st>::iterator model_it;
    bool found;

    for (i=0; i<world_->ModelCount(); ++i) {
        found = false;
        for (model_it = parsed_models_.begin(); model_it!=parsed_models_.end(); ++model_it) {
            if (world_->ModelByIndex(i)->GetName() == model_it->name)
                found = true;
        }
        if (!found && !(world_->ModelByIndex(i)->IsStatic())) // model not in listand not static, parse it for potential buoyancy flags
            ParseNewModel(world_->ModelByIndex(i));
    }

    // look for deleted world models
    model_it = parsed_models_.begin();
    while (model_it != parsed_models_.end()) {
        found = false;
        for (i=0; i<world_->ModelCount(); ++i) {
            if (world_->ModelByIndex(i)->GetName() == model_it->name)
                found = true;
        }
        if (!found) // model name not in world anymore, remove the corresponding links
            RemoveDeletedModel(model_it);
        else
            ++model_it;
    }

    // here buoy_links is up-to-date with the links that are subject to buoyancy, let's apply it
    ignition::math::Vector3d actual_force, cob_position, velocity_difference, torque;
    double signed_distance_to_surface;
    for (std::vector<link_st>::iterator link_it = buoyant_links_.begin(); link_it!=buoyant_links_.end(); ++link_it) {
        // get world position of the center of buoyancy
        cob_position = link_it->link->WorldPose().Pos() + link_it->link->WorldPose().Rot().RotateVector(link_it->buoyancy_center);
        // start from the theoretical buoyancy force
        actual_force = link_it->buoyant_force;
        if (has_surface_) {
            // adjust force depending on distance to surface (very simple model)
            signed_distance_to_surface = surface_plane_.W()
                                         - surface_plane_.X() * cob_position.X()
                                         - surface_plane_.Y() * cob_position.Y()
                                         - surface_plane_.Z() * cob_position.Z();
            if (signed_distance_to_surface > -link_it->limit) {
                if (signed_distance_to_surface > link_it->limit) {
                    actual_force *= 0;
                    return;
                } else {
                    actual_force *= cos(M_PI/4.*(signed_distance_to_surface/link_it->limit + 1));
                }
            }
        }

        // get velocity damping
        // linear velocity difference in the link frame
        velocity_difference = link_it->link->WorldPose().Rot().RotateVectorReverse(link_it->link->WorldLinearVel() - fluid_velocity_);
        // to square
        velocity_difference.X()*= fabs(velocity_difference.X());
        velocity_difference.Y()*= fabs(velocity_difference.Y());
        velocity_difference.Z()*= fabs(velocity_difference.Z());
        // apply damping coefficients
        actual_force -= link_it->link->WorldPose().Rot().RotateVector(link_it->linear_damping * velocity_difference);

        link_it->link->AddForceAtWorldPosition(actual_force, cob_position);

        // same for angular damping
        velocity_difference = link_it->link->RelativeAngularVel();
        velocity_difference.X()*= fabs(velocity_difference.X());
        velocity_difference.Y()*= fabs(velocity_difference.Y());
        velocity_difference.Z()*= fabs(velocity_difference.Z());
        link_it->link->AddRelativeTorque(-link_it->angular_damping*velocity_difference);

        ignition::math::Vector3d vec;
        ignition::math::Pose3d pose;
    }
}



void FreeBuoyancyPlugin::ParseNewModel(const physics::ModelPtr &_model) {
    // define new model structure: name / pointer / publisher to odometry
    model_st new_model;
    new_model.name = _model->GetName();
    new_model.model_ptr = _model;
    // tells this model has been parsed
    parsed_models_.push_back(new_model);

    const unsigned int previous_link_number = buoyant_links_.size();
    std::string urdf_content;

    // parse actual URDF as XML (that's ugly) to get custom buoyancy tags
    // links from urdf
    TiXmlDocument urdf_doc;
    urdf_doc.Parse(urdf_content.c_str(), 0);

    const ignition::math::Vector3d WORLD_GRAVITY = world_->Gravity();

    TiXmlElement* urdf_root = urdf_doc.FirstChildElement();
    TiXmlElement* link_test;
    TiXmlNode* urdf_node, *link_node, *buoy_node;
    double compensation;
    unsigned int link_index;
    physics::LinkPtr sdf_link;
    bool found;

    for (auto sdf_element = _model->GetSDF()->GetFirstElement(); sdf_element != 0; sdf_element = sdf_element->GetNextElement()) {
        urdf_doc.Parse(sdf_element->ToString("").c_str(), 0);
        urdf_root = urdf_doc.FirstChildElement();
        if (sdf_element->HasElement("link")) {
            auto link = sdf_element->GetElement("link");
            auto linkName = link->GetAttribute("name")->GetAsString();

            if (link->HasElement("buoyancy")) {
                found = true;
                link_test = (new TiXmlElement(link->ToString("")));
                link_node = link_test->Clone();
                sdf_link = _model->GetChildLink(linkName);

                for (auto buoy = link->GetElement("buoyancy"); buoy != NULL; buoy = buoy->GetNextElement()) {

                    // this link is subject to buoyancy, create an instance
                    link_st new_buoy_link;
                    new_buoy_link.model_name = _model->GetName();            // in case this model is deleted
                    new_buoy_link.link =  sdf_link;    // to apply forces
                    new_buoy_link.limit = .1;

                    // get data from urdf
                    // default values
                    new_buoy_link.buoyancy_center = sdf_link->GetInertial()->CoG();
                    new_buoy_link.linear_damping = new_buoy_link.angular_damping = 5 * ignition::math::Vector3d::One * sdf_link->GetInertial()->Mass();

                    compensation = 0;

                    if (buoy->HasElement("origin")) {
                        auto vec = buoy->GetElement("origin")->GetAttribute("xyz")->GetAsString();
                        ReadVector3(vec, new_buoy_link.buoyancy_center);
                    }
                    if (buoy->HasElement("compensation")) {
                        compensation = stof(buoy->GetElement("compensation")->GetValue()->GetAsString());
                    }

                    new_buoy_link.buoyant_force = -compensation * sdf_link->GetInertial()->Mass() * WORLD_GRAVITY;

                    // store this link
                    buoyant_links_.push_back(new_buoy_link);
                }
            }
        }
    }

    if (!urdf_root) {
        return;
    }
    for (urdf_node = urdf_root->FirstChild(); urdf_node != 0; urdf_node = urdf_node->NextSibling()) {
        if (found) {
            for (; link_node != 0; link_node = link_node->NextSibling()) {
                if (link_node->ValueStr() == "buoyancy") {
                    // this link is subject to buoyancy, create an instance
                    link_st new_buoy_link;
                    new_buoy_link.model_name = _model->GetName();            // in case this model is deleted
                    new_buoy_link.link =  sdf_link;    // to apply forces
                    new_buoy_link.limit = .1;

                    // get data from urdf
                    // default values
                    new_buoy_link.buoyancy_center = sdf_link->GetInertial()->CoG();
                    new_buoy_link.linear_damping = new_buoy_link.angular_damping = 5 * ignition::math::Vector3d::One * sdf_link->GetInertial()->Mass();

                    compensation = 0;
                    for (buoy_node = link_node->FirstChild(); buoy_node != 0; buoy_node = buoy_node->NextSibling()) {
                        if (buoy_node->ValueStr() == "origin")
                            ReadVector3((buoy_node->ToElement()->Attribute("xyz")), new_buoy_link.buoyancy_center);
                        else if (buoy_node->ValueStr() == "compensation")
                            compensation = atof(buoy_node->ToElement()->GetText());
                        else if (buoy_node->ValueStr() == "limit") {
                            std::stringstream ss(buoy_node->ToElement()->Attribute("radius"));
                            ss >> new_buoy_link.limit;
                        } else if (buoy_node->ValueStr() == "damping") {
                            if (buoy_node->ToElement()->Attribute("xyz") != NULL) {
                                ReadVector3((buoy_node->ToElement()->Attribute("xyz")), new_buoy_link.linear_damping);
                                cout << ("Found linear damping\n");
                            }
                            if (buoy_node->ToElement()->Attribute("rpy") != NULL) {
                                ReadVector3((buoy_node->ToElement()->Attribute("rpy")), new_buoy_link.angular_damping);
                                cout << ("Found angular damping\n");
                            }
                        } else
                            cout << ("Unknown tag <%s/> in buoyancy node for model %s\n", buoy_node->ValueStr().c_str(), _model->GetName().c_str());
                    }

                    new_buoy_link.buoyant_force = -compensation * sdf_link->GetInertial()->Mass() * WORLD_GRAVITY;

                    // store this link
                    buoyant_links_.push_back(new_buoy_link);
                }
            }   // out of loop: buoyancy-related nodes
        }       // out of condition: in sdf
    }           // out of loop: all urdf nodes
    if (previous_link_number == buoyant_links_.size()) {
        cout << "Buoyancy plugin: " << "No links subject to buoyancy inside " << _model->GetName().c_str() << ("\n");
    } else {
        cout << "Buoyancy plugin: " << "Added " << (int) buoyant_links_.size()-previous_link_number << " buoy links from " << _model->GetName().c_str() << ("\n");
    }
}

void FreeBuoyancyPlugin::RemoveDeletedModel(std::vector<model_st>::iterator &_model_it) {
    cout << ("Removing deleted model: %s\n", _model_it->name.c_str());

    // remove model stored links
    std::vector<link_st>::iterator link_it = buoyant_links_.begin();
    while (link_it != buoyant_links_.end()) {
        if (link_it->model_name == _model_it->name)
            link_it = buoyant_links_.erase(link_it);
        else
            ++link_it;
    }

    // remove it from the list
    _model_it = parsed_models_.erase(_model_it);
}

}
