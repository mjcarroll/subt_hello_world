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

#ifndef ARTIFACT_LOCALIZATION_EXTERNAL_POINT_RADAR_HH_
#define ARTIFACT_LOCALIZATION_EXTERNAL_POINT_RADAR_HH_

#include <unordered_map>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <mutex>
#include <std_msgs/String.h>

class RobotPositionRadar
{
public:
  /// \brief Constructor
  RobotPositionRadar();

  /// \brief Check if is there any point on a position, whithin the tolerance.
  /// \param[in] msg The location to be checked.
  /// \param[in] tolerance The tolerance of the check.
  bool CheckPointWithinRadar(const geometry_msgs::Point & msg, const float & tolerance);

private:
  /// \brief Receives the position of other robots close to this one.
  /// \param[in] msg The position received by the controller
  void ExternalRobotPointCB(const std_msgs::String::ConstPtr & msg);

  /// \brief Deletes the point when its lifetime is over.
  /// \param[in] name The point to be deleted.
  void TimerCallback(const std::string & name);

  /// \brief Nodehandle to subscribe to the location topic and create timers
  ros::NodeHandle nh_;

  /// \brief List of points alive.
  std::unordered_map<std::string, geometry_msgs::Point> radar_points_;

  /// \brief List of timers for the points.
  std::unordered_map<std::string, ros::Timer> radar_timers_;

  /// \brief Subscriber for getting the position of other robots.
  ros::Subscriber position_subscriber_;

  /// \brief Mutex to separete deleting from checking points.
  std::mutex points_mutex_;
};

#endif
