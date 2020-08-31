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

#ifndef SUBT_ROBOT_CONTROL_CONTROLLER_H_
#define SUBT_ROBOT_CONTROL_CONTROLLER_H_

#include <graph_controller/controller.hh>

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <subt_solution_comms/comms_manager.hh>
#include <tf2_ros/transform_listener.h>

/// \class Controller controller.hh
class Controller
{
  /// \brief Contstructor
  public: Controller();

  /// \brief Waits for simulation and then starts
  private: void PerformStartAction();

  /// \brief Stops all running services and terminates.
  private: void PerformStopAction();

  /// \brief Callback that is triggered whenever a new message is published to the battery topic
  private: void BatteryCallback(const sensor_msgs::BatteryState & msg);

  /// \brief Wait for artifact origin transformation to be published
  /// \param[in] map_frame The name of the map frame
  private: void WaitForOrigin(const std::string & map_frame);

  /// \brief Shares the robot's latest pose in the artifact origin frame
  private: void SharePose(const ros::TimerEvent&);

  /// \brief Called if the graph controller begins backtracking early
  /// (usually happens if there are no more places to explore)
  void BacktrackStarted();

  /// \brief The name of the robot
  std::string robot_name_;

  /// \brief The name of the artifact origin frame
  std::string artifact_origin_frame_;

  /// \brief ROS nodehandle
  private: ros::NodeHandle nh_;

  /// \brief ROS nodehandle used for retrieving ROS params
  private: ros::NodeHandle private_nh_;

  /// \brief Service client that sends requests to report artifacts
  private: ros::ServiceClient artifact_report_client_;

  /// \brief Defines if the robot has odometry topic
  bool has_odom_;

  /// \brief Subscriber for robot's battery state msg
  ros::Subscriber battery_sub_;

  /// \brief Robot's battery percentage threshold for initiating back routine
  float min_battery_percentage_;

  /// \brief Backtracking state
  bool back_track_state_;

  /// \brief used to share a robot's pose information
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher pose_publisher_;
  ros::Timer pose_timer_;

  /// \brief Handles navigation/exploration for a robot
  private: graph_controller::Controller *graph_controller_;

  /// \brief Handles communications with the base station and other robots
  private: CommsManager *comms_manager_;

  /// \brief Sets the backtrack state to true after a certain amount of
  /// time if not set to true already
  private: ros::Timer backtrack_timer_;
};

#endif
