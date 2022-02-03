#ifndef _MOVING_TARGET_HH_
#define _MOVING_TARGET_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Rand.hh>

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>

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
      this->initPose = this->model->GetWorldPose();
    }

    void OnUpdate()
    {
        std::ifstream infile;
        infile.open("./plugins/moving_target/toggle_movement.txt");
        if (infile.is_open())
        {
            infile >> this->isMoving;
            infile.close();
        }

        infile.open("./plugins/moving_target/reset_position.txt");
        if (infile.is_open())
        {
            infile >> this->isReset;
            if (this->isReset) {
               /* ResetPosition(); */
               this->model->SetWorldPose(this->initPose);
               /* std::ofstream outfile; */
               /* outfile.open("./plugins/moving_target/reset_position.txt", std::ofstream::out | std::ofstream::trunc); */
               /* outfile << "0\n"; */
               /* outfile.close(); */
               this->isReset = false;
            }
            infile.close();
        }

        if (this->isMoving) {

            double max_y = 20.0, min_y = -max_y, 
                   max_x = 12.5, min_x = -max_x;
            gazebo::math::Pose pose = this->model->GetWorldPose();
            double x_now = pose.pos.x;
            double y_now = pose.pos.y;

            double distance_travelled = sqrt( pow(x_now-this->x_prev,2) + pow(y_now-this->y_prev,2) );

            if (distance_travelled > 1) {
                this->ChangeDirection();
                this->x_prev = x_now;
                this->y_prev = y_now;
            } 
            else if ( this->x == x_now && this->y == y_now )
                this->ChangeDirection();
            else if (x_now < min_x)
                this->ChangeDirection({north,northeast,northwest});
            else if (x_now > max_x)
                this->ChangeDirection({south,southeast,southwest});
            else if (y_now < min_y)
                this->ChangeDirection({northwest,west,southwest});
            else if (y_now > max_y)
                this->ChangeDirection({northeast,east,southeast});
           
            /* if ( this->x == x_now && this->y == y_now ) { */
            /*     RandomizeSpeed(); */
            /* } else if (x_now < -2.5) { */
            /*     RandomizeSpeed(0, 1, -1, 1); */
            /* } else if (x_now > 2.5) { */
            /*     RandomizeSpeed(-1, 0, -1, 1); */
            /* } else if (y_now < -4.5) { */
            /*     RandomizeSpeed(-1, 1, 0, 1); */
            /* } else if (y_now > 4.5) { */
            /*     RandomizeSpeed(-1, 1, -1, 0); */
            /* } */

            this->MoveModelsPlane(this->dx, this->dy, 0, 0, 0, 0);

            this->x = x_now;
            this->y = y_now;
        } else {
            this->MoveModelsPlane(0, 0, 0, 0, 0, 0);
        }
    }

    enum Direction { 
        northwest,
        north, 
        northeast, 
        east,
        southeast,
        south, 
        southwest,
        west,
        size 
    };

    void ChangeDirection(const std::vector<Direction>& dirs = {}) 
    {
        Direction dir;
        if (dirs.size() == 0) {
            double chances = ignition::math::Rand::DblUniform(0, 1);

            if (chances < 0.7) {
                dir = Direction(0);
            } else {
                int randint = ignition::math::Rand::IntUniform(1, Direction::size);
                dir = Direction(randint); 
            }
        } else {
            int randint = ignition::math::Rand::IntUniform(0, dirs.size());
            dir = dirs[randint];
        }
        Move(dir);
    }

    void Move(Direction dir)
    {
        double dist = 0.7;
        double diag = dist; // sqrt(2*pow(dist,2)); 

        switch (dir) {
            case Direction::north:
                this->dx = dist;
                this->dy = 0;
                break;
            case Direction::northeast:
                this->dx = diag;
                this->dy = -diag;
                break;
            case Direction::east:
                this->dx = 0;
                this->dy = -dist;
                break;
            case Direction::southeast:
                this->dx = -diag;
                this->dy = -diag;
                break;
            case Direction::south:
                this->dx = -dist;
                this->dy = 0;
                break;
            case Direction::southwest:
                this->dx = -diag;
                this->dy = diag;
                break;
            case Direction::west:
                this->dx = 0;
                this->dy = dist;
                break;
            case Direction::northwest:
                this->dx = diag;
                this->dy = diag;
                break;
        }
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

    private: bool isMoving = false, isReset = false;
    private: double x, y, x_prev, y_prev;
    private: gazebo::math::Pose initPose;
    private: float dx, dy;
    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(MovingTarget)
}
#endif
