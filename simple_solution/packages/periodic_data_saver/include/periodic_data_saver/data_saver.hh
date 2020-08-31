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

#ifndef PERIODIC_DATA_SAVER__DATA_SAVER_HH_
#define PERIODIC_DATA_SAVER__DATA_SAVER_HH_

#include <ros/ros.h>

namespace periodic_data_saver
{

/// \class DataSaver data_saver.hh
/// \brief Class for periodically saving large data (like maps) from a simulation run.
/// This mimics the behavior of a rostopic relay, but only relays a message every "save_rate" seconds
/// ROS params:
/// \param map_topic_original The original topic that is receiving map messages.
/// Default = "map" (string)
/// \param trajectory_topic_original The original topic that is receiving trajectory_list messages.
/// Default = "trajectory_node_list" (string)
/// \param save_rate How frequently (in seconds) the topics will be polled for their latest message to be saved.
/// Default = 300 (int), which is 5 minutes
/// \param map_topic_logging The topic which the periodically polled map data is sent to for logging
/// Default = "/robot_data/map" (string)
/// \param trajectory_topic_logging The topic which the periodically polled trajectory_list data is sent to for logging
/// Default = "/robot_data/trajectory_list" (string)
class DataSaver
{
  /// \brief Constructor
  /// \param[in] use_timer A boolean that defines whether a timer should be started or not.
  /// This value should be set to false if this class is being used to save data at a rate/number of times
  /// that is different than the periodic saving behavior of a timer.
  public: DataSaver(const bool use_timer=true);

  /// \brief Waits for the next message(s) to become available from ROS topic(s)
  /// and saves this message(s) to a logging/rosbag topic
  public: void SaveData();

  /// \brief Callback that calls the SaveData() method periodically
  private: void SaverCB();

  /// \brief ROS nodehandle
  private: ros::NodeHandle nh_;

  /// \brief ROS nodehandle with a private namespace. Used for retrieving parameters
  private: ros::NodeHandle private_nh_;

  /// \brief ROS timer object that is used to trigger the SaverCB() method at a pre-defined rate
  private: ros::Timer timer_;

  /// \brief ROS publisher that publishes a map message to a logging topic
  private: ros::Publisher map_pub_;

  /// \brief ROS publisher that publishes a trajectory_list message to a logging topic
  private: ros::Publisher trajectory_list_pub_;

  /// \brief The name of the original (high frequency) map topic that will be periodically polled
  private: std::string map_topic_original_;

  /// \brief The name of the original (high frequency) trajectory_list topic that will be periodically polled
  private: std::string trajectory_topic_original_;
};
}  // namespace periodic_data_saver

#endif  // PERIODIC_DATA_SAVER__DATA_SAVER_HH_
