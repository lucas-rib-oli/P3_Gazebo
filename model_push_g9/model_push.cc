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

        // Listen to the update event. This event is broadcast every simulation iteration.
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    void OnUpdate()
    {
        // Apply a small linear velocity to the model.
        model->SetLinearVel(ignition::math::Vector3d(0.3, 1.0, 0));
        model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.0));
        ignition::math::Pose3d pose = model->WorldPose();
        printf("At: %f %f %f\n", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
        std::cout << "Heading: " << pose.Rot().Yaw() << std::endl;

        ignition::math::Vector3d destination_point ( 4.0, 1.0, 0 );

        double angle_to_goal = std::atan ( (destination_point.Y() - pose.Pos().Y()) / (destination_point.X() - pose.Pos().X()) );

    }

private:
    physics::ModelPtr model; // Pointer to the model
    event::ConnectionPtr updateConnection; // Pointer to the update event connection
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
