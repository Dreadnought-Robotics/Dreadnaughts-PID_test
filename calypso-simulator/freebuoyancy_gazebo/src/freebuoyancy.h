#pragma once

#include <gazebo/common/Plugin.hh>
#include <ignition/math.hh>




namespace gazebo {

class FreeBuoyancyPlugin : public  ModelPlugin {
public:

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnUpdate();

private:
    struct link_st {
        std::string model_name;
        physics::LinkPtr link;
        ignition::math::Vector3d buoyant_force;
        ignition::math::Vector3d buoyancy_center;
        ignition::math::Vector3d linear_damping;
        ignition::math::Vector3d angular_damping;
        double limit;
    };

    struct model_st {
        std::string name;
        physics::ModelPtr model_ptr;
    };

    // parse a Vector3 string
    void ReadVector3(const std::string &_string, ignition::math::Vector3d &_vector);
    // parse a new model
    void ParseNewModel(const physics::ModelPtr &_model);
    // removes a deleted model
    void RemoveDeletedModel(std::vector<model_st>::iterator &_model_it);

private:
    // plugin options
    bool has_surface_;
    ignition::math::Vector4d surface_plane_;
    std::string description_;

    physics::WorldPtr world_;
    event::ConnectionPtr update_event_;

    // links that are subject to fluid effects
    std::vector<link_st> buoyant_links_;
    // models that have been parsed
    std::vector<model_st> parsed_models_;

    ignition::math::Vector3d fluid_velocity_;

};

GZ_REGISTER_MODEL_PLUGIN(FreeBuoyancyPlugin)
}
