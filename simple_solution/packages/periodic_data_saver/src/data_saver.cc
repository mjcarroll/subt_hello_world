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

#include <periodic_data_saver/data_saver.hh>

#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

using namespace periodic_data_saver;

DataSaver::DataSaver(const bool use_timer) :
  private_nh_("~")
{
  this->private_nh_.param("map_topic_original", this->map_topic_original_, std::string("map"));
  this->private_nh_.param("trajectory_topic_original", this->trajectory_topic_original_, std::string("trajectory_node_list"));

  int save_rate;
  std::string map_topic_logging, trajectory_topic_logging;
  this->private_nh_.param("save_rate", save_rate, 300);
  this->private_nh_.param("map_topic_logging", map_topic_logging, std::string("/robot_data/map"));
  this->private_nh_.param("trajectory_topic_logging", trajectory_topic_logging, std::string("/robot_data/trajectory_list"));

  if (use_timer)
  {
    this->timer_ = this->nh_.createTimer(ros::Duration(save_rate), boost::bind(&DataSaver::SaverCB, this));
  }

  this->map_pub_ = this->nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_logging, 1, true);
  this->trajectory_list_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(trajectory_topic_logging, 1, true);
}

void DataSaver::SaveData()
{
  auto map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(this->map_topic_original_);
  this->map_pub_.publish(map_msg);

  auto trajectory_list_msg = ros::topic::waitForMessage<visualization_msgs::MarkerArray>(this->trajectory_topic_original_);
  this->trajectory_list_pub_.publish(trajectory_list_msg);
}

void DataSaver::SaverCB()
{
  this->SaveData();
}
