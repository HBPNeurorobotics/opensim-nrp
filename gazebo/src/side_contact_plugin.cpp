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

#include "side_contact_plugin.h"
#include <ignition/math/Vector3.hh>

#include "SimulationLogger.h"

using namespace gazebo;
//GZ_REGISTER_MODEL_PLUGIN(SideContactPlugin)

/////////////////////////////////////////////////
SideContactPlugin::SideContactPlugin() : ModelPlugin(), m_loggingActive(false)
{
}

/////////////////////////////////////////////////
SideContactPlugin::~SideContactPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void SideContactPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Model pointer is null");

  if (!_sdf->HasElement("contact_sensor_name"))
  {
    gzerr << "'contact_sensor_name' not specified in SDF\n";
  }
  this->model = _model;
  this->world = this->model->GetWorld();
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());

  this->contactSensorName = _sdf->Get<std::string>("contact_sensor_name");
  bool sensorFound = this->FindContactSensor();
  if (!sensorFound || !this->parentSensor)
  {
    gzerr << "Contact sensor not found: " << this->contactSensorName << "\n";
  }

  std::string parentLinkName = this->parentLink->GetScopedName();
  std::string defaultCollisionName = parentLinkName + "::__default__";
  if (this->parentSensor->GetCollisionCount() != 1)
  {
    gzerr << "SideContactPlugin requires a single collision to observe contacts for\n";
    return;
  }

  this->collisionName = this->parentSensor->GetCollisionName(0);
  if (this->collisionName == defaultCollisionName)
  {
    // Use the first collision of the parent link by default
    if (this->parentLink->GetCollisions().empty())
    {
      gzerr << "Couldn't find any collisions for the contact sensor.";
      return;
    }
    unsigned int index = 0;
    this->collisionName = this->parentLink->GetCollision(index)->GetScopedName();
  }
  gzdbg << "[" << this->model->GetName() << "] Watching collisions on: " << this->collisionName << "\n";

  if (_sdf->HasElement("contact_side_normal"))
  {
    this->sideNormal = _sdf->Get<ignition::math::Vector3d>("contact_side_normal");
  }
  else
  {
    this->sideNormal = ignition::math::Vector3d(0, 0, 1);
  }

  if (_sdf->HasElement("update_rate"))
  {
    std::string ur = _sdf->Get<std::string>("update_rate");
    try
    {
      double v = std::stod(ur);
      if (v <= 0)
      {
        gzerr << "Illegal update_rate value [" << v << "]" << std::endl;
      }
      this->updateRate = v;
    } catch (const std::exception& e)
    {
      gzerr << "Unable to parse update_rate [" << ur << "]" << std::endl;
    }
  }

  if (_sdf->HasElement("log_directory"))
  {
    this->logDirectory = _sdf->Get<std::string>("log_directory");
  }

  if (_sdf->HasElement("logging_active"))
  {
    this->m_loggingActive = _sdf->Get<bool>("logging_active");
  }

  this->lastUpdateTime = this->world->GetSimTime();

  // FIXME: how to not hard-code this gazebo prefix?
  std::string contactTopic = "/gazebo/" + this->scopedContactSensorName;
  boost::replace_all(contactTopic, "::", "/");
  this->contactSub =
    this->node->Subscribe(contactTopic, &SideContactPlugin::OnContactsReceived, this);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&SideContactPlugin::OnUpdate, this, _1));
}

void SideContactPlugin::Init()
{
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  if (!ros::isInitialized())
  {
    gzthrow("SideContactPlugin failed to initialize ROS!");
  }

  m_nh.reset(new ros::NodeHandle());

  std::string topic_name = "/" + model->GetName() + "/cmd_logging";
  m_loggingSub = m_nh->subscribe<std_msgs::Bool>(topic_name, 1,
                                     boost::bind(&SideContactPlugin::OnLoggingCommand, this, _1));

  m_logger = new SimulationLogger(this->logDirectory);
}

void SideContactPlugin::Fini()
{
  m_loggingSub.shutdown();
  m_nh->shutdown();

  delete m_logger;
  m_logger = NULL;
}

void SideContactPlugin::OnLoggingCommand(const std_msgs::Bool::ConstPtr& _msg)
{
  bool loggingActive = _msg->data;
  gzdbg << "Received logging control command: Switch to " << loggingActive << "\n";

  if (m_loggingActive != loggingActive)
    this->contactForceLoggers.clear();

  m_loggingActive = loggingActive;
}

/////////////////////////////////////////////////
bool SideContactPlugin::FindContactSensor()
{
  auto sensorManager = sensors::SensorManager::Instance();
  auto links = this->model->GetLinks();
  for (const auto &link : links)
  {
    std::string scopedContactSensorName =
      this->world->GetName() + "::" + link->GetScopedName() + "::" + this->contactSensorName;
    for (unsigned int i = 0; i < link->GetSensorCount(); ++i)
    {
      if (link->GetSensorName(i) == scopedContactSensorName)
      {
        this->parentLink = link;
        this->scopedContactSensorName = scopedContactSensorName;
        this->parentSensor =
          std::static_pointer_cast<sensors::ContactSensor>(
            sensorManager->GetSensor(scopedContactSensorName));
        return this->parentSensor != 0;
      }
    }
  }
  return false;
}

/////////////////////////////////////////////////
void SideContactPlugin::OnContactsReceived(ConstContactsPtr& _msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  this->newestContactsMsg = *_msg;
  this->newMsg = true;

  lock.unlock();
}

/////////////////////////////////////////////////
void SideContactPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  this->CalculateContactingModels();
}

/////////////////////////////////////////////////
void SideContactPlugin::CalculateContactingLinks()
{
  boost::mutex::scoped_lock lock(this->mutex);

  if (!this->newMsg)
  {
    return;
  }

  this->contactingLinks.clear();

  // Get all the contacts
  for (int i = 0; i < this->newestContactsMsg.contact_size(); ++i)
  {
    // Get the collision that's not the parent link
    const auto &contact = this->newestContactsMsg.contact(i);
    const std::string *collision = &(contact.collision1());
    if (this->collisionName == *collision) {
      collision = &(contact.collision2());
    }

    physics::CollisionPtr collisionPtr =
      boost::static_pointer_cast<physics::Collision>(this->world->GetEntity(*collision));
    if (collisionPtr) { // ensure the collision hasn't been deleted
      this->contactingLinks.insert(collisionPtr->GetLink());
    }
  }

  if (m_loggingActive)
  {
    LogContactData();
  }

  this->newMsg = false;
}

/////////////////////////////////////////////////
void SideContactPlugin::CalculateContactingModels()
{
  this->CalculateContactingLinks();
  this->contactingModels.clear();
  for (auto link : this->contactingLinks)
  {
    physics::ModelPtr model = link->GetModel();
    this->contactingModels.insert(model);
  }
}

/////////////////////////////////////////////////
bool SideContactPlugin::TimeToExecute()
{
  // We're using a custom update rate.
  if (this->updateRate <= 0)
    return true;

  gazebo::common::Time curTime = this->world->GetSimTime();
  auto dt = (curTime - this->lastUpdateTime).Double();
  if (dt < 0)
  {
    // Probably we had a reset.
    this->lastUpdateTime = curTime;
    return false;
  }

  // Update based on sensorsUpdateRate.
  if (dt < (1.0 / this->updateRate))
    return false;

  this->lastUpdateTime = curTime;
  return true;
}

void SideContactPlugin::LogContactData()
{
  try
  {
    for (int k = 0; k < this->newestContactsMsg.contact_size(); ++k)
    {
      const gazebo::msgs::Contact& ct_data = this->newestContactsMsg.contact(k);
      std::string collision_pair_id = ct_data.collision1() + "_" + ct_data.collision2();
      std::string log_file_name;
      std::ofstream log_stream;

      std::stringstream data_log_stream;

      bool open_append = true;
      if (contactForceLoggers.find(collision_pair_id) == contactForceLoggers.end())
      {
        log_file_name = this->logDirectory + "/ContactData_" + collision_pair_id;
        open_append = false;
        contactForceLoggers.insert(std::make_pair(collision_pair_id, log_file_name));

        std::string data_description("Contact point data for " + collision_pair_id);
        m_logger->newLogger(collision_pair_id, log_file_name, data_description);
      }
      else
      {
        log_file_name = contactForceLoggers[collision_pair_id];
      }

      for (int l = 0; l < ct_data.wrench_size(); ++l)
      {
        if (l < ct_data.depth_size() && l < ct_data.wrench_size())
        {
          std::stringstream log_file_name_local;
          log_file_name_local << log_file_name << l << ".log";

          if (open_append)
            log_stream.open(log_file_name_local.str(), std::ios::out | std::ios::app | std::ios::ate);
          else
            log_stream.open(log_file_name_local.str(), std::ios::out);

          if (log_stream.is_open())
          {

              const gazebo::msgs::Wrench& b1_w = ct_data.wrench(l).body_1_wrench();
              const gazebo::msgs::Wrench& b2_w = ct_data.wrench(l).body_2_wrench();

              ignition::math::Vector3d b1_f = gazebo::msgs::ConvertIgn(b1_w.force());
              ignition::math::Vector3d b1_t = gazebo::msgs::ConvertIgn(b1_w.torque());
              ignition::math::Vector3d b2_f = gazebo::msgs::ConvertIgn(b2_w.force());
              ignition::math::Vector3d b2_t = gazebo::msgs::ConvertIgn(b2_w.torque());

#ifdef SIDE_CONTACT_PLUGIN_GAZEBO_8_SUPPORT
              log_stream << this->world->SimTime().Double() << " " << ct_data.depth(l) << " " << b1_f.Length() << " " << b2_f.Length() << " " << b1_t.Length() << " " << b2_t.Length() << "\n";
              data_log_stream << << this->world->SimTime().Double() << " " << ct_data.depth(l) << " " << b1_f.Length() << " " << b2_f.Length() << " " << b1_t.Length() << " " << b2_t.Length() << "\n";
#else // SIDE_CONTACT_PLUGIN_GAZEBO_8_SUPPORT
              log_stream << this->world->GetSimTime().Double() << " " << ct_data.depth(l) << " " << b1_f.Length() << " " << b2_f.Length() << " " << b1_t.Length() << " " << b2_t.Length() << "\n";
              data_log_stream << this->world->GetSimTime().Double() << " " << ct_data.depth(l) << " " << b1_f.Length() << " " << b2_f.Length() << " " << b1_t.Length() << " " << b2_t.Length() << "\n";
#endif // SIDE_CONTACT_PLUGIN_GAZEBO_8_SUPPORT

              m_logger->logData(collision_pair_id, data_log_stream.str());

              log_stream.close();


          }
        }
      }
    }
  }
  catch (google::protobuf::FatalException& ex)
  {
    gzerr << "Caught Protobuf exception in contact logging: " << ex.message() << " (file: " << ex.filename() << ", line: " << ex.line() << ")" << std::endl;
  }
}
