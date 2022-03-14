#ifndef _STATIC_TARGET_HH_
#define _STATIC_TARGET_HH_

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
  class StaticTarget : public ModelPlugin
  {
    /// \brief Constructor
    public: StaticTarget() {}

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
        /* std::cerr << std::stoi(id) << std::endl; */

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
    /* std::vector<std::vector<std::string>> readCSV(std::istream &in) { */
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
                /* table.push_back(fields); */
        }
        /* return table; */
    }


    void Init()
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
              boost::bind(&StaticTarget::OnUpdate, this));
      this->initPose = this->model->GetWorldPose();
    }

    void OnUpdate()
    {
        std::ifstream infile;
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
                    /* std::cerr << row[1] << std::endl; */
                    this->initPose.pos.x = std::stod(row[1]);
                    this->initPose.pos.y = std::stod(row[2]);
                    datafile.close();
                }
               this->model->SetWorldPose(this->initPose);
            }
            infile.close();
        }
    }

    private: 
    int id;
    bool isMoving = false, isReset = false;
    double x, y, x_prev, y_prev;
    gazebo::math::Pose initPose;
    float dx, dy;
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(StaticTarget)
}
#endif
