#ifndef _MOVING_TARGET_HH_
#define _MOVING_TARGET_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Rand.hh>

#include <iostream>
#include <fstream>
#include <string>

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
    }

    void Init()
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
              boost::bind(&MovingTarget::OnUpdate, this));
    }

    void OnUpdate()
    {
        gazebo::math::Pose pose = this->model->GetWorldPose();
        double x_now = pose.pos.x;
        double y_now = pose.pos.y;

        std::ifstream file ("./plugins/moving_target/toggle_movement.txt");
        if (file.is_open())
        {
            file >> this->isMoving;
            file.close();
        }

        if (this->isMoving) {
            if ( this->x == x_now && this->y == y_now ) {
                RandomizeSpeed();
            } else if (x_now < -2.5) {
                RandomizeSpeed(0, 1, -1, 1);
            } else if (x_now > 2.5) {
                RandomizeSpeed(-1, 0, -1, 1);
            } else if (y_now < -4.5) {
                RandomizeSpeed(-1, 1, 0, 1);
            } else if (y_now > 4.5) {
                RandomizeSpeed(-1, 1, -1, 0);
            }

            this->MoveModelsPlane(this->dx, this->dy, 0, 0, 0, 0);
        } else {
            this->MoveModelsPlane(0, 0, 0, 0, 0, 0);
        }

        this->x = x_now;
        this->y = y_now;
    }

    void RandomizeSpeed(int min_dx=-1, int max_dx=1, int min_dy=-1, int max_dy=1) 
    {
        this->dx = ignition::math::Rand::DblUniform(min_dx, max_dx);
        this->dy = ignition::math::Rand::DblUniform(min_dy, max_dy);
    }

    void MoveModelsPlane(float linear_x_vel, float linear_y_vel, float linear_z_vel, float angular_x_vel, float angular_y_vel, float angular_z_vel)
    {

        std::string model_name = this->model->GetName();

        this->model->SetLinearVel(ignition::math::Vector3d(linear_x_vel, linear_y_vel, linear_z_vel));
        this->model->SetAngularVel(ignition::math::Vector3d(angular_x_vel, angular_y_vel, angular_z_vel));

    }

    private: bool isMoving = false;
    private: double x, y;
    private: float dx, dy;
    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(MovingTarget)
}
#endif
