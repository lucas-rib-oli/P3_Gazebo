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
#include <stdio.h>
#include <unistd.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{

class ModelPush : public ModelPlugin
{
public:
    
    double angle_to_goal = 0.0;
    bool once = true;

    std::vector <ignition::math::Vector3d> path_points;
    int count = 0;
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        model = _parent;

        // --------------------- read points for map --------------------- //
        std::string line;
		char s[100];
		// recursividad de carpetas 
		chdir("..");
        std::string method = "best";
        std::string path_2_points ( getcwd (s, 100) );
		path_2_points += "/gazebo-tools-master/"; // concatenar ruta
        
		// Ruta hacia los txt donde se almacenan los puntos resultantes de los algoritmos de path planning
		std::ifstream file_points ( path_2_points + method + "_points.txt" );
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
		}
		else
		{
			std::cout << "No se ha podido abrir el archivo: " << path_2_points + method + "_points.txt | se ejecutara el modo por defecto" << std::endl;
			method = "default";
            this->path_points.push_back ( ignition::math::Vector3d ( 4.5, 6.5, 0 ) );
            this->path_points.push_back ( ignition::math::Vector3d ( 7.5, 6.5, 0 ) );
            this->path_points.push_back ( ignition::math::Vector3d ( 7.5, 8.5, 0 ) );
            this->path_points.push_back ( ignition::math::Vector3d ( 12.5, 8.5, 0 ) );
            this->path_points.push_back ( ignition::math::Vector3d ( 12.5, 14.5, 0 ) );
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
        ignition::math::Vector3d destination_point = this->path_points [count];

        this->angle_to_goal = std::atan2 ( (destination_point.Y() - pose.Pos().Y()), (destination_point.X() - pose.Pos().X()) );
        
        if ( angle_to_goal - heading > 0.05  )
        {
            model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 1.0));
        }
        else if ( angle_to_goal - heading < -0.05 )
        {
            model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, -1.0));
        }   
        else if ( this->get_euclidean_distance ( pose.Pos(), destination_point ) > 0.1 && count < this->path_points.size() )
        {
            pose = model->WorldPose();
            model->SetLinearVel(ignition::math::Vector3d(1.0 * std::cos (angle_to_goal), 1.0 * std::sin ( angle_to_goal ), 0));
        }
        else
        {
            model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0));
            count ++;
        }

        if ( count == this->path_points.size())
        {
            std::cout << "We are in the goal !!" << std::endl;
            exit (0);
        }
        this->once = false;
        
    }

private:
    physics::ModelPtr model; // Pointer to the model
    event::ConnectionPtr updateConnection; // Pointer to the update event connection
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
