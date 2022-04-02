#ifndef _MOVING_TARGET_HH_
#define _MOVING_TARGET_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Rand.hh>

#include <iostream>
#include <istream>
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

        std::string myname = _model->GetName();
        std::string id = "";

        for(int i=0; i<myname.length(); i++) {
            if(isdigit(myname[i]))
                id += myname[i];
        }

        this->id = std::stoi(id);
    }

    enum class CSVState {
        UnquotedField,
        QuotedField,
        QuotedQuote
    };

    std::vector<std::string> readCSVRow(const std::string &row) {
        CSVState state = CSVState::UnquotedField;
        std::vector<std::string> fields {""};
        size_t i = 0; // index of the current field
        for (char c : row) {
            switch (state) {
                case CSVState::UnquotedField:
                    switch (c) {
                        case ',': // end of field
                                  fields.push_back(""); i++;
                                  break;
                        case '"': state = CSVState::QuotedField;
                                  break;
                        default:  fields[i].push_back(c);
                                  break; }
                    break;
                case CSVState::QuotedField:
                    switch (c) {
                        case '"': state = CSVState::QuotedQuote;
                                  break;
                        default:  fields[i].push_back(c);
                                  break; }
                    break;
                case CSVState::QuotedQuote:
                    switch (c) {
                        case ',': // , after closing quote
                                  fields.push_back(""); i++;
                                  state = CSVState::UnquotedField;
                                  break;
                        case '"': // "" -> "
                                  fields[i].push_back('"');
                                  state = CSVState::QuotedField;
                                  break;
                        default:  // end of quote
                                  state = CSVState::UnquotedField;
                                  break; }
                    break;
            }
        }
        return fields;
    }

    /// Read CSV file, Excel dialect. Accept "quoted fields ""with quotes"""
    std::vector<std::string> readCSV(std::istream &in) {
        std::vector<std::vector<std::string>> table;
        std::string row;
        while (!in.eof()) {
            std::getline(in, row);
            if (in.bad() || in.fail()) {
                break;
            }
            auto fields = readCSVRow(row);
            int id = std::stoi(fields[0]);
            if(id == this->id)
                return fields;
        }
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
        infile.open("./comm/toggle_movement.txt");
        if (infile.is_open())
        {
            infile >> this->isMoving;
            infile.close();
        }

        infile.open("./comm/reset_position.txt");
        if (infile.is_open())
        {
            infile >> this->isReset;
            if (this->isReset) {
                std::ifstream datafile;
                datafile.open("./comm/positions.csv");
                if (datafile.is_open())
                {
                    auto row = readCSV(datafile);
                    this->initPose.pos.x = std::stod(row[1]);
                    this->initPose.pos.y = std::stod(row[2]);
                    datafile.close();
                }
               this->model->SetWorldPose(this->initPose);
            }
            infile.close();
        }

        gazebo::math::Pose pose = this->model->GetWorldPose();
        double x_now = pose.pos.x;
        double y_now = pose.pos.y;
        if (this->isMoving) {
            if (this->hasSavedPosition)
                this->hasSavedPosition = false;

            double max_y = 8.0, min_y = -max_y, 
                   max_x = 5.0, min_x = -max_x;

            double distance_travelled = sqrt( pow(x_now-this->x_prev,2) + pow(y_now-this->y_prev,2) );

            if (distance_travelled > 0.4) {
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
           
            this->MoveModelsPlane(this->dx, this->dy, 0, 0, 0, 0);

            this->x = x_now;
            this->y = y_now;
        } else {
            this->MoveModelsPlane(0, 0, 0, 0, 0, 0);
            if (!this->hasSavedPosition) {
                std::string filename = "./comm/position_" + std::to_string(id) + ".csv"; 
                std::ofstream outfile(filename, std::ios::trunc);
                if (outfile.is_open()) {
                    outfile << x_now << "," << y_now;
                    outfile.close();
                    this->hasSavedPosition = true;
                }
            }
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
        double dist = 0.38;
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

    void MoveModelsPlane(float linear_x_vel, float linear_y_vel, float linear_z_vel, float angular_x_vel, float angular_y_vel, float angular_z_vel)
    {

        std::string model_name = this->model->GetName();

        this->model->SetLinearVel(ignition::math::Vector3d(linear_x_vel, linear_y_vel, linear_z_vel));
        this->model->SetAngularVel(ignition::math::Vector3d(angular_x_vel, angular_y_vel, angular_z_vel));
    }

    private: 
    int id;
    bool isMoving = false, isReset = false, hasSavedPosition = false;
    double x, y, x_prev, y_prev;
    gazebo::math::Pose initPose;
    float dx, dy;
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(MovingTarget)
}
#endif
