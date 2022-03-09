/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>
#include <vector>
#include <memory>
#include <stdio.h>
#include <unistd.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/sensors/sensors.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"


#define METHOD "breadth" // method to use. Available: best, a_star, breadth, depth, laser, default
#define GOAL_X 12.5 // Coordenada X de la meta
#define GOAL_Y 14.5 // Coordenada Y de la meta

void cb(ConstLaserScanStampedPtr & _msg)
{
    std::cout << "In callback" << std::endl;
    std::cout << _msg->DebugString() << std::endl;
}

namespace gazebo
{

class ModelPush : public ModelPlugin
{
public:
    
    double angle_to_goal = 0.0;
    std::vector <ignition::math::Vector3d> path_points;
    int count = 0;

    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {        
        // Store the pointer to the model
        model = _parent;

        // --------------------- read points for map --------------------- //
        if ( this->_method != "laser" && this->_method != "default" )
        {
            std::string line;
            char s[100];
            // recursividad de carpetas 
            chdir("..");
            std::string path_2_points ( getcwd (s, 100) );
            path_2_points += "/gazebo-tools-master/"; // concatenar ruta
            
            // Ruta hacia los txt donde se almacenan los puntos resultantes de los algoritmos de path planning
            std::ifstream file_points ( path_2_points + _method + "_points.txt" );
            std::vector <std::vector <double>> coords;
            if ( file_points.is_open() )
            {
                while (std::getline (file_points, line))
                {
                    int start = 0;
                    int end = line.find(" ");
                    line.substr (0, end - start);
                    std::string space_delimiter = " ";

                    size_t pos = 0;
                    std::vector <double> coord;
                    while ((pos = line.find(space_delimiter)) != std::string::npos) 
                    {
                        coord.push_back( std::stold( line.substr(0, pos) ) + 0.5 ); // Se guardan los puntos del fichero
                        line.erase(0, pos + space_delimiter.length());
                    }
                    coord.push_back ( std::stold( line ) + 0.5 );
                    coords.push_back ( coord );
                }
                // Invertir vector
                std::vector<double> aux; 
                for (size_t i = 0; i < coords.size()/2; i++)
                {
                    aux = coords[i]; 
                    coords[i] = coords[coords.size()-1 -i];
                    coords[coords.size()-1 -i] = aux;
                }
                for (size_t i = 0; i < coords.size(); i++)
                {
                    this->path_points.push_back ( ignition::math::Vector3d ( coords[i][0], coords[i][1], 0.0 ) );
                }

                this->path_points.erase ( this->path_points.begin() ); // Remove start point
                // ----------------------------------------------------------- //
            }
            else
            {
                std::cout << "No se ha podido abrir el archivo: " << path_2_points + _method + "_points.txt | se ejecutara el modo por defecto" << std::endl;
                _method = "default";
                this->path_points.push_back ( ignition::math::Vector3d ( 4.5, 6.5, 0 ) );
                this->path_points.push_back ( ignition::math::Vector3d ( 7.5, 6.5, 0 ) );
                this->path_points.push_back ( ignition::math::Vector3d ( 7.5, 8.5, 0 ) );
                this->path_points.push_back ( ignition::math::Vector3d ( 12.5, 8.5, 0 ) );
                this->path_points.push_back ( ignition::math::Vector3d ( 12.5, 14.5, 0 ) );
            }
        }

        // Listen to the update event. This event is broadcast every simulation iteration.
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
    }

    double get_euclidean_distance ( ignition::math::Vector3d p1, ignition::math::Vector3d p2)
    {
        return std::sqrt ( std::pow ( p2.X() - p1.X(), 2 ) + std::pow ( p2.Y() - p1.Y(), 2 ) );
    }

    // Called by the world update start event
    void OnUpdate()
    {
        // Apply a small linear velocity to the model.
        model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0));
        model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.0));
        ignition::math::Pose3d pose = model->WorldPose();
        double heading = pose.Rot().Yaw();
        this->angle_to_goal = std::atan2 ( (_destination_point.Y() - pose.Pos().Y()), (_destination_point.X() - pose.Pos().X()) );
        // best, a_star, breadth, depth & default
        if (this->_method == "a_star" || this->_method == "best" || this->_method == "breadth" || this->_method == "depth" || this->_method == "default" )
        {
            _destination_point = this->path_points [count];
            
            if ( count == this->path_points.size())
            {
                std::cout << "Se ha llegado al punto de destino !!" << std::endl;
                model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0));
                model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.0));
            }
            else if ( angle_to_goal - heading > 0.05  )
            {
                model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 1.0));
            }
            else if ( angle_to_goal - heading < -0.05 )
            {
                model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, -1.0));
            }   
            else if ( this->get_euclidean_distance ( pose.Pos(), _destination_point ) > 0.1 && count < this->path_points.size() )
            {
                pose = model->WorldPose();
                model->SetLinearVel(ignition::math::Vector3d(1.0 * std::cos (angle_to_goal), 1.0 * std::sin ( angle_to_goal ), 0));
            }
            else
            {
                model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0));
                count ++;
            }
            
        } else
        {
            // get ray sensor
            sensors::SensorPtr model_sensor = sensors::SensorManager::Instance()->GetSensor("default::pioneer::hokuyo::link::laser");
            sensors::RaySensorPtr ray_sensor = std::dynamic_pointer_cast<sensors::RaySensor>(model_sensor);

            _destination_point = ignition::math::Vector3d ( GOAL_X, GOAL_Y, 0 );
            ray_sensor->Ranges ( _ranges_vector );

            if ( _ranges_vector.size() > 0 )
            {
                if ( this->get_euclidean_distance ( pose.Pos(), _destination_point ) < 0.1 )
                {
                    model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0));
                    model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.0));
                    std::cout << "Se ha llegado al punto de destino !!" << std::endl;
                }
                else if ( this->some_in_min_value ( -85.0, 0.0, 0.25 ) ) // obstaculo delante derecha
                {
                    model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0));
                    model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 2.0));
                }
                else if ( this->all_in_min_value ( -92.0, -88.0, 0.6 ) ) // obstaculo derecha
                {
                    model->SetLinearVel(ignition::math::Vector3d(2.0 * std::cos (heading), 2.0 * std::sin ( heading ), 0));
                    model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
                }
                else if (this->some_in_min_value ( 0.0, 85.0, 0.25 ))
                {
                    model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0));
                    model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, -2.0));
                }
                else if ( this->all_in_min_value ( 88.0, 92.0, 0.6 ) ) // obstaculo izquierda
                {
                    model->SetLinearVel(ignition::math::Vector3d(2.0 * std::cos (heading), 2.0 * std::sin ( heading ), 0));
                    model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
                }
                else if ( angle_to_goal - heading > 0.05  )
                {
                    model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0));
                    model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 2.0));
                }
                else if ( angle_to_goal - heading < -0.05 )
                {
                    model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0));
                    model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, -2.0));
                }
                else
                {
                    model->SetLinearVel(ignition::math::Vector3d(1.0 * std::cos (angle_to_goal), 1.0 * std::sin ( angle_to_goal ), 0));
                    model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
                }
            }
        }
        
    }

    bool all_in_min_value ( float start_angle, float end_angle, float distance = 0.3 )
    {
        int start = static_cast<int> ( (std::abs(start_angle + 130)) / this->_laser_resolution ) - 1;
        int end = static_cast<int> ( (std::abs(end_angle + 130)) / this->_laser_resolution ) - 1;
        for (size_t i = start; i <= end; i++)
        {
            if ( _ranges_vector[i] > distance )
            {
                return false;
            }
        }
        return true; // todos a una distance menor que 0.3
    }

    bool some_in_min_value ( float start_angle, float end_angle, float distance = 0.3 )
    {
        
        int start = static_cast<int> ( (std::abs(start_angle + 130)) / this->_laser_resolution ) - 1;
        int end = static_cast<int> ( (std::abs(end_angle + 130)) / this->_laser_resolution ) - 1;
        for (int i = start; i <= end; i++)
        {
            if ( _ranges_vector[i] <= distance )
            {
                return true;
            }
        }
        return false; // todos a una distance mayor que 0.3
    }

private:
    physics::ModelPtr model; // Pointer to the model
    event::ConnectionPtr updateConnection; // Pointer to the update event connection
    transport::NodePtr gzNode;

    std::string _method = METHOD;


    ignition::math::Vector3d _destination_point;
    std::vector <double> _ranges_vector;

    float _laser_resolution = 260.0 / 640.0; // [º/por laser]

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
