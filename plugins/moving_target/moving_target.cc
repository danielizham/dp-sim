#ifndef _MOVING_TARGET_HH_
#define _MOVING_TARGET_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class MovingTarget : public ModelPlugin
  {
    /// \brief Constructor
    public: MovingTarget() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->model = _model;
      // Just output a message for now
      std::cerr << "\nThe target plugin is attach to model[" <<
        _model->GetName() << "]\n";

      this->MoveModelsPlane(10, 0, 0, 0, 0, 0);
    }

    void MoveModelsPlane(float linear_x_vel, float linear_y_vel, float linear_z_vel, float angular_x_vel, float angular_y_vel, float angular_z_vel)
    {

        std::string model_name = this->model->GetName();

        /* ROS_DEBUG("Moving model=%s",model_name.c_str()); */

        this->model->SetLinearVel(ignition::math::Vector3d(linear_x_vel, linear_y_vel, linear_z_vel));
        this->model->SetAngularVel(ignition::math::Vector3d(angular_x_vel, angular_y_vel, angular_z_vel));

        /* ROS_DEBUG("Moving model=%s....END",model_name.c_str()); */

    }

    private: physics::ModelPtr model;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(MovingTarget)
}
#endif
