/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <artifact_localization/ExternalPointRadar.hh>
#include <ignition/msgs/pose.pb.h>

RobotPositionRadar::RobotPositionRadar():
position_subscriber_(this->nh_.subscribe("other_robot_pose", 10, &RobotPositionRadar::ExternalRobotPointCB, this))
{
}

void RobotPositionRadar::ExternalRobotPointCB(const std_msgs::String::ConstPtr & msg)
{
  const std::lock_guard<std::mutex> lock(this->points_mutex_);
  ignition::msgs::Pose pose;
  if (!pose.ParseFromString(msg->data))
  {
    ROS_ERROR("RobotPositionRadar::ExternalRobotPointCB: error deserializing message.");
  }
  geometry_msgs::Point position;
  position.x = pose.position().x();
  position.y = pose.position().y();
  position.z = pose.position().z();
  std::string name = pose.name();
  if (this->radar_points_.find(name) != this->radar_points_.end())
  {
    this->radar_points_.erase(name);
  }
  this->radar_points_.insert({name, position});
  if (this->radar_timers_.find(name) != this->radar_timers_.end())
  {
    this->radar_timers_.erase(name);
  }
  ros::Timer timer = this->nh_.createTimer(ros::Duration(2.0), boost::bind(&RobotPositionRadar::TimerCallback, this, name), true);
  this->radar_timers_.insert({name, timer});
}

void RobotPositionRadar::TimerCallback(const std::string & name)
{
  const std::lock_guard<std::mutex> lock(this->points_mutex_);
  this->radar_points_.erase(name);
  this->radar_timers_.erase(name);
}

bool RobotPositionRadar::CheckPointWithinRadar(const geometry_msgs::Point & msg, const float & tolerance)
{
  const std::lock_guard<std::mutex> lock(this->points_mutex_);
  for (std::unordered_map<std::string, geometry_msgs::Point>::iterator it = this->radar_points_.begin(); it != this->radar_points_.end(); ++it)
  {
    geometry_msgs::Point temp = it->second;
    if (sqrt(pow(msg.x - temp.x, 2.0) + pow(msg.y - temp.y, 2.0) + pow(msg.z - temp.z, 2.0)) < tolerance)
    {
      return true;
    }
  }
  return false;
}
