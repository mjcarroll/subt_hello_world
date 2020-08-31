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

#ifndef FRONTIER_FINDER__COSTMAP_CLIENT_HH_
#define FRONTIER_FINDER__COSTMAP_CLIENT_HH_ 

#include <string>

#include <costmap_2d/costmap_2d.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/PoseStamped.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace frontier_finder 
{

class CostmapClient
{
 public:
  CostmapClient(const tf2_ros::Buffer* tf_buffer);

  CostmapClient(const tf2_ros::Buffer* tf_buffer,
     std::string costmap_topic, std::string footprint_topic,
     std::string costmap_updates_topic, std::string robot_base_frame,
     double transform_tolerance);

  costmap_2d::Costmap2D* getCostmap();

  geometry_msgs::PoseStamped getRobotPose() const;

  const std::string& getGlobalFrameID() const;

 private:
  void updateFullMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void updatePartialMap(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);

  costmap_2d::Costmap2D costmap_;
  const tf2_ros::Buffer * const tf_buffer_;
  double transform_tolerance_;

  ros::Subscriber costmap_sub_;
  ros::Subscriber costmap_updates_sub_;

  std::string global_frame_;
  std::string robot_base_frame_;
};
}  // namespace frontier_finder

#endif  // FRONTIER_FINDER__COSTMAP_CLIENT_HH_ 
