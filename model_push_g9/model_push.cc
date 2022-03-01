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
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{

class ModelPush : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        model = _parent;

        this->goal_points.push_back ( ignition::math::Vector3d ( 4.5, 6.5, 0 ) );
        this->goal_points.push_back ( ignition::math::Vector3d ( 7.5, 6.5, 0 ) );
        this->goal_points.push_back ( ignition::math::Vector3d ( 7.5, 8.5, 0 ) );
        this->goal_points.push_back ( ignition::math::Vector3d ( 12.5, 8.5, 0 ) );
        this->goal_points.push_back ( ignition::math::Vector3d ( 12.5, 14.5, 0 ) );

        // Listen to the update event. This event is broadcast every simulation iteration.
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
    }

    double get_euclidean_distance ( ignition::math::Vector3d p1, ignition::math::Vector3d p2)
    {
        return std::sqrt ( std::pow ( p2.X() - p1.X(), 2 ) + std::pow ( p2.Y() - p1.Y(), 2 ) );
    }

    double angle_to_goal = 0.0;
    bool once = true;

    std::vector <ignition::math::Vector3d> goal_points;
    int count = 0;

    // Called by the world update start event
    void OnUpdate()
    {
        // Apply a small linear velocity to the model.
        model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0));
        model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.0));
        ignition::math::Pose3d pose = model->WorldPose();
        double heading = pose.Rot().Yaw();
        ignition::math::Vector3d destination_point = this->goal_points [count];

        this->angle_to_goal = std::atan ( (destination_point.Y() - pose.Pos().Y()) / (destination_point.X() - pose.Pos().X()) );
        std::cout << "Heading: " << heading << std::endl;
        std::cout << "angle_to_goal: " << angle_to_goal << std::endl;
        std::cout << "Current position: " << pose.Pos().X() << " " << pose.Pos().Y() << std::endl;
        
           
        if ( get_euclidean_distance ( pose.Pos(), destination_point ) > 0.1 && count < this->goal_points.size() )
        {
            pose = model->WorldPose();
            model->SetLinearVel(ignition::math::Vector3d(1.0 * std::cos (angle_to_goal), 1.0 * std::sin ( angle_to_goal ), 0));
        }
        else
        {
            model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0));
            count ++;
        }

        if ( count == this->goal_points.size())
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
