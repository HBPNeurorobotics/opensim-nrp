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
/*
 * Desc: Conveyor Belt Plugin
 * Author: Deanna Hood
 */
#ifndef _GAZEBO_CONVEYOR_BELT_PLUGIN_HH_
#define _GAZEBO_CONVEYOR_BELT_PLUGIN_HH_

#include <string>

#include "side_contact_plugin.h"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/util/system.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <iostream>

#if GAZEBO_MAJOR_VERSION >= 8 && GAZEBO_MINOR_VERSION > 0
#define CONVEYOR_BELT_PLUGIN_GAZEBO_8_SUPPORT
#endif

namespace gazebo
{
  /// \brief A plugin for a conveyor belt.
  class GAZEBO_VISIBLE ConveyorBeltPlugin : public SideContactPlugin
  {
    /// \brief Constructor.
    public: ConveyorBeltPlugin();

    /// \brief Destructor.
    public: virtual ~ConveyorBeltPlugin();

    /// \brief Load the model plugin.
    /// \param[in] _model Pointer to the model that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    public: void Init();

    public: void Fini();

    /// \brief Callback that receives the world update event
    protected: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Callback for responding to control commands
    protected: void OnControlCommand(const std_msgs::Float64::ConstPtr& _msg);

    /// \brief Axis for belt velocity in local frame (+Y by default)
    protected: ignition::math::Vector3d velocityAxis;

    /// \brief Belt velocity (m/s)
    protected: double beltVelocity;

    /// \brief Mutex to protect the belt velocity
    protected: std::mutex mutex;

    /// \brief Set the state of the conveyor belt
    public: void SetVelocity(double velocity);

    /// \brief Generate a scoped topic name from a local one
    /// \param local local topic name
    protected: std::string Topic(std::string topicName) const;

    /// \brief Act on links that are ontop of the belt
    protected: void ActOnContactingLinks(double velocity);

    protected: ros::NodeHandlePtr m_nh;
    protected: ros::Subscriber m_velocitySub;

    protected: std::map<std::string, std::string> jointTorqueLoggers;
  };
}
#endif

