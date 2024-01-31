/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
 * Desc: Contact plugin
 * Author: Nate Koenig mod by John Hsu
 *
 * Adapted by Petr Štibinger (stibipet@fel.cvut.cz)
 */

#include "gazebo_lidar_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <boost/algorithm/string.hpp>
#include <common.h>
#include <ignition/math/Rand.hh>
#include <tf2/LinearMath/Quaternion.h>

using namespace gazebo;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(LidarPlugin)

/////////////////////////////////////////////////
LidarPlugin::LidarPlugin()
{
}

/////////////////////////////////////////////////
LidarPlugin::~LidarPlugin()
{
  newLaserScansConnection_->~Connection();
  newLaserScansConnection_.reset();
  parentSensor_.reset();
}

/////////////////////////////////////////////////
void LidarPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  parentSensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!parentSensor_)
    gzthrow("LidarPlugin requires a Ray Sensor as its parent");

  world_ = physics::get_world(parentSensor_->WorldName());

  newLaserScansConnection_ = parentSensor_->LaserShape()->ConnectNewLaserScans(
      boost::bind(&LidarPlugin::OnNewLaserScans, this));

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_lidar_plugin] Please specify a robotNamespace.\n";

  if (_sdf->HasElement("simulate_fog")) {
    simulate_fog_ = _sdf->GetElement("simulate_fog")->Get<bool>();
  } else {
    simulate_fog_ = false;
  }
  // get minimum distance
  if (_sdf->HasElement("minDistance")) {
    min_distance_ = _sdf->GetElement("minDistance")->Get<double>();
    if (min_distance_ < kSensorMinDistance) {
      min_distance_ = kSensorMinDistance;
    }
  } else {
    gzwarn << "[gazebo_lidar_plugin]: Missing param 'minDistance', defaulting to " << kDefaultMinDistance << "\n";
    min_distance_ = kDefaultMinDistance;
  }

  // get maximum distance
  if (_sdf->HasElement("maxDistance")) {
    max_distance_ = _sdf->GetElement("maxDistance")->Get<double>();
    if (max_distance_ > kSensorMaxDistance) {
      max_distance_ = kSensorMaxDistance;
    }
  } else {
    gzwarn << "[gazebo_lidar_plugin]: Missing param 'maxDistance', defaulting to " << kDefaultMaxDistance << "\n";
    max_distance_ = kDefaultMaxDistance;
  }

  // Set high and low signal strength
  // The considered relationship of distance to returned signal strength is an
  // inverse square.
  low_signal_strength_ = sqrt(min_distance_);
  high_signal_strength_ = sqrt(max_distance_ + 0.02); // extend the threshold so there is still quality at max distance

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Get the root model name
  const string scopedName = _parent->ParentName();
  vector<string> names_splitted;
  boost::split(names_splitted, scopedName, boost::is_any_of("::"));
  names_splitted.erase(std::remove_if(begin(names_splitted), end(names_splitted),
                            [](const string& name)
                            { return name.size() == 0; }), end(names_splitted));
  std::string rootModelName = names_splitted.front(); // The first element is the name of the root model

  // the second to the last name is the model name
  const std::string parentSensorModelName = names_splitted.rbegin()[1];

  // get lidar topic name
  if(_sdf->HasElement("topic")) {
    lidar_topic_ = _sdf->GetElement("topic")->Get<std::string>();
  } else {
    // if not set by parameter, get the topic name from the model name
    lidar_topic_ = parentSensorModelName;
    gzwarn << "[gazebo_lidar_plugin]: " + names_splitted.front() + "::" + names_splitted.rbegin()[1] +
      " using lidar topic \"" << parentSensorModelName << "\"\n";
  }

  if (_sdf->HasElement("x")){
    x_ = _sdf->GetElement("x")->Get<double>();
  } else {
    gzwarn << "[gazebo_lidar_plugin]: Missing param 'x', defaulting to 0" << std::endl;
    x_ = 0;
  }

  if (_sdf->HasElement("y")){
    y_ = _sdf->GetElement("y")->Get<double>();
  } else {
    gzwarn << "[gazebo_lidar_plugin]: Missing param 'y', defaulting to 0" << std::endl;
    y_ = 0;
  }

  if (_sdf->HasElement("z")){
    z_ = _sdf->GetElement("z")->Get<double>();
  } else {
    gzwarn << "[gazebo_lidar_plugin]: Missing param 'z', defaulting to 0" << std::endl;
    z_ = 0;
  }

  if (_sdf->HasElement("roll")){
    roll_ = _sdf->GetElement("roll")->Get<double>();
  } else {
    gzwarn << "[gazebo_lidar_plugin]: Missing param 'roll', defaulting to 0" << std::endl;
    roll_ = 0;
  }

  if (_sdf->HasElement("pitch")){
    pitch_ = _sdf->GetElement("pitch")->Get<double>();
  } else {
    gzwarn << "[gazebo_lidar_plugin]: Missing param 'pitch', defaulting to 0" << std::endl;
    pitch_ = 0;
  }

  if (_sdf->HasElement("yaw")){
    yaw_ = _sdf->GetElement("yaw")->Get<double>();
  } else {
    gzwarn << "[gazebo_lidar_plugin]: Missing param 'yaw', defaulting to 0" << std::endl;
    yaw_ = 0;
  }

  if (_sdf->HasElement("parentFrameName")){
    parent_frame_name_ = _sdf->GetElement("parentFrameName")->Get<std::string>();
  } else {
    gzwarn << "[gazebo_lidar_plugin]: Missing param 'parentFrameName', defaulting to 'world'" << std::endl;
    parent_frame_name_ = "world";
  }

  if (_sdf->HasElement("frameName")){
    frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
  } else {
    gzwarn << "[gazebo_lidar_plugin]: Missing param 'frame_name', defaulting to 'sensor'" << std::endl;
    frame_name_ = "sensor";
  }

  // Calculate parent sensor rotation WRT `base_link`
  const ignition::math::Quaterniond q_ls = parentSensor_->Pose().Rot();

  // Set the orientation
  orientation_.set_x(q_ls.X());
  orientation_.set_y(q_ls.Y());
  orientation_.set_z(q_ls.Z());
  orientation_.set_w(q_ls.W());

  rosnode_ = new ros::NodeHandle(namespace_);
  tf_pub_ = rosnode_->advertise<tf2_msgs::TFMessage>("/tf_gazebo_static", 100, true);
  createStaticTransforms();
  timer_ = rosnode_->createWallTimer(ros::WallDuration(1.0), &LidarPlugin::publishStaticTransforms, this);


  // start lidar topic publishing
  lidar_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Range>("~/" + names_splitted[0] + "/link/" + lidar_topic_, 10);
}

/////////////////////////////////////////////////
void LidarPlugin::OnNewLaserScans()
{
  // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world_->SimTime();
#else
  common::Time now = world_->GetSimTime();
#endif

  lidar_message_.set_time_usec(now.Double() * 1e6);
  lidar_message_.set_min_distance(min_distance_);
  lidar_message_.set_max_distance(max_distance_);

  // get current distance measured from the sensor
  double current_distance = parentSensor_->Range(0);

  // set distance to min/max if actual value is smaller/bigger
  if (simulate_fog_ && current_distance > 2.0f) {
    double whiteNoise = ignition::math::Rand::DblNormal(0.0f, 0.1f);
    current_distance = 2.0f + whiteNoise;
  } else if (current_distance < min_distance_ || std::isinf(current_distance)) {
    current_distance = min_distance_;
  } else if (current_distance > max_distance_) {
    current_distance = max_distance_;
  }


  lidar_message_.set_current_distance(current_distance);
  lidar_message_.set_h_fov(kDefaultFOV);
  lidar_message_.set_v_fov(kDefaultFOV);
  lidar_message_.set_allocated_orientation(new gazebo::msgs::Quaternion(orientation_));

  // Compute signal strength
  // Other effects like target size, shape or reflectivity are not considered
  const double signal_strength = sqrt(current_distance);

  // Compute and set the signal quality
  // The signal quality is normalized between 1 and 100 using the absolute
  // signal strength (DISTANCE_SENSOR signal_quality value of 0 means invalid)
  uint8_t signal_quality = 1;
  if (signal_strength > low_signal_strength_) {
    signal_quality = static_cast<uint8_t>(99 * ((high_signal_strength_ - signal_strength) /
            (high_signal_strength_ - low_signal_strength_)) + 1);
  }

  lidar_message_.set_signal_quality(signal_quality);

  lidar_pub_->Publish(lidar_message_);
}

/////////////////////////////////////////////////
void LidarPlugin::createStaticTransforms() {
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp            = ros::Time(0);
  static_transformStamped.header.frame_id         = parent_frame_name_;
  static_transformStamped.child_frame_id          = frame_name_;
  static_transformStamped.transform.translation.x = x_;
  static_transformStamped.transform.translation.y = y_;
  static_transformStamped.transform.translation.z = z_;
  tf2::Quaternion quat;
  quat.setRPY(roll_, pitch_, yaw_);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();

  this->tf_message_.transforms.push_back(static_transformStamped);
  /* ROS_INFO_NAMED("2dlidar", "created"); */
}

/////////////////////////////////////////////////
void LidarPlugin::publishStaticTransforms([[maybe_unused]] const ros::WallTimerEvent &event) {
  this->tf_pub_.publish(this->tf_message_);
}
