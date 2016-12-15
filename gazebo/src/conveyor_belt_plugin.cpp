/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <boost/algorithm/string/replace.hpp>
#include <string>

#include "conveyor_belt_plugin.h"
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ConveyorBeltPlugin)

/////////////////////////////////////////////////
ConveyorBeltPlugin::ConveyorBeltPlugin() : SideContactPlugin()
{
}

/////////////////////////////////////////////////
ConveyorBeltPlugin::~ConveyorBeltPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->parentSensor.reset();
  this->world.reset();
}

//////////////////////////////////////////////////
std::string ConveyorBeltPlugin::Topic(std::string topicName) const
{
  std::string globalTopicName = "~/";
  globalTopicName += this->parentSensor->Name() + "/" + this->GetHandle() + "/" + topicName;
  boost::replace_all(globalTopicName, "::", "/");

  return globalTopicName;
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  SideContactPlugin::Load(_model, _sdf);

  if (this->updateRate > 0)
    gzdbg << "ConveyorBeltPlugin running at " << this->updateRate << " Hz\n";
  else
    gzdbg << "ConveyorBeltPlugin running at the default update rate\n";

  if (_sdf->HasElement("belt_start_velocity"))
  {
    this->beltVelocity = _sdf->Get<double>("belt_start_velocity");
  }
  else {
    this->beltVelocity = 0.5;
  }
  gzdbg << "Using belt start velocity of: " << this->beltVelocity << " m/s\n";

  if (_sdf->HasElement("velocity_axis"))
  {
    this->velocityAxis = _sdf->Get<ignition::math::Vector3d>("velocity_axis");
  }
  else
  {
    gzerr << "'velocity_axis' tag not found\n";
  }
}

void ConveyorBeltPlugin::Init()
{
  SideContactPlugin::Init();

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_conveyor_belt", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  if (!ros::isInitialized())
  {
    gzthrow("ConveyorBeltPlugin failed to initialize ROS!");
  }

  m_nh.reset(new ros::NodeHandle());

  std::string topic_name = "/" + model->GetName() + "/cmd_vel";
  m_velocitySub = m_nh->subscribe<std_msgs::Float64>(topic_name, 1,
                                     boost::bind(&ConveyorBeltPlugin::OnControlCommand, this, _1));
}

void ConveyorBeltPlugin::Fini()
{
  SideContactPlugin::Fini();

  m_velocitySub.shutdown();
  m_nh->shutdown();
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  double velocity;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    velocity = this->beltVelocity;
  }
  this->ActOnContactingLinks(velocity);

  // If we're using a custom update rate value we have to check if it's time to
  // update the plugin or not.
  if (!this->TimeToExecute())
    return;

  auto prevNumberContactingLinks = this->contactingLinks.size();
  this->CalculateContactingLinks();

  if (prevNumberContactingLinks != this->contactingLinks.size()) {
    gzdbg << "Number of links on top of belt: " << this->contactingLinks.size() << "\n";
  }

  SideContactPlugin::CalculateContactingModels();

  // gzdbg << "Contacting models: " << this->contactingModels.size() << "\n";
  // gzdbg << "Contacting links : " << this->contactingLinks.size() << "\n";
  for (auto cm : this->contactingModels)
  {
    physics::ModelPtr model = cm;
    const physics::Joint_V& joints = model->GetJoints();
    // gzdbg << "Log model data for model: " << model->GetName() << "\n";
    for (int k = 0; k < joints.size(); ++k)
    {
      physics::JointPtr joint = joints[k];
      std::ofstream log_stream;
      std::string log_file_name;
      if (jointTorqueLoggers.find(joint->GetName()) == jointTorqueLoggers.end())
      {
        log_file_name = this->logDirectory + "/JointData_" + joint->GetName() + ".log";
        // gzdbg << "  First time log for joint: " << log_file_name << "\n";
        jointTorqueLoggers.insert(std::make_pair(joint->GetName(), log_file_name));
        log_stream.open(log_file_name, std::ios::out);
        if (log_stream.is_open())
        {
          // gzdbg << "   Log file stream open, writing." << "\n";
          physics::JointWrench wrench = joint->GetForceTorque(0);
          log_stream << this->world->SimTime().Double() << " " << wrench.body1Force.GetLength() << " " << wrench.body2Force.GetLength()
                     << " " << wrench.body1Torque.GetLength() << " " << wrench.body2Torque.GetLength() << "\n";
          log_stream.close();
        }
      }
      else
      {
        log_file_name = jointTorqueLoggers[joint->GetName()];
        // gzdbg << "  Append log for joint: " << log_file_name << "\n";
        log_stream.open(log_file_name, std::ios::out | std::ios::app | std::ios::ate);
        if (log_stream.is_open())
        {
          // gzdbg << "   Log file stream open, writing." << "\n";
          physics::JointWrench wrench = joint->GetForceTorque(0);
          log_stream << this->world->SimTime().Double() << " " << wrench.body1Force.GetLength() << " " << wrench.body2Force.GetLength()
                     << " " << wrench.body1Torque.GetLength() << " " << wrench.body2Torque.GetLength() << "\n";
          log_stream.close();
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::ActOnContactingLinks(double velocity)
{
  ignition::math::Vector3d velocity_beltFrame = velocity * this->velocityAxis;
  auto beltPose = this->parentLink->GetWorldPose().Ign();
  math::Vector3 velocity_worldFrame = beltPose.Rot().RotateVector(velocity_beltFrame);
  for (auto linkPtr : this->contactingLinks)
  {
    if (linkPtr)
    {
      linkPtr->SetLinearVel(velocity_worldFrame);
    }
  }
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::OnControlCommand(const std_msgs::Float64::ConstPtr& _msg)
{
  double requestedVelocity = _msg->data;
  gzdbg << "Received control command of: " << requestedVelocity << "\n";
  this->SetVelocity(requestedVelocity);
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::SetVelocity(double velocity)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  gzdbg << "Setting velocity to: " << velocity << "\n";
  this->beltVelocity = velocity;
}
