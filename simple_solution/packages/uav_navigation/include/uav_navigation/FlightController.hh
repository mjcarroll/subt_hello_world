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

#ifndef UAV_NAVIGATION_FLIGHT_CONTROLLER_H_
#define UAV_NAVIGATION_FLIGHT_CONTROLLER_H_

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/TwistStamped.h"

/// \brief Approximate time policy for the message synchronizer,
/// which allows for syncing of both top & bottom lidar data,
/// even if the sensors process data at slightly different times.
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::LaserScan, sensor_msgs::LaserScan> ApproximationSyncPolicy;

/// \brief States the flight controller can be in
/// \param kReset resetting the UAV to a specific height above the floor
/// \param kMaintain holding the UAV at a specific height above the floor
enum HoverState {kReset, kMaintain};

/// \class FlightController FlightController.hh
/// \brief Class for controlling the UAV's flight altitude
class FlightController
{
  /// \brief Constructor
  /// \param[in] topLidarTopic the topic for incoming messages from the upward-facing point lidar
  /// (sensor_msgs/LaserScan)
  /// \param[in] bottomLidarTopic the topic for incoming messages from the downward-facing point lidar
  /// (sensor_msgs/LaserScan)
  /// \param[in] cmdVelTopic the topic for publishing command velocities that help the UAV hover
  /// (geometry_msgs/Twist)
  /// \param[in] resetStatusTopic the topic for publishing if the controller is in the kReset state or not
  /// (uav_navigation_msgs/ResetStatusMsg)
  public: FlightController(
    const std::string & topLidarTopic = "top_scan",
    const std::string & bottomLidarTopic = "bottom_scan",
    const std::string & cmdVelTopic = "controller_cmd_vel",
    const std::string & resetStatusTopic = "reset_state_status"
  );

  /// \brief Callback that takes in time-synchronized messages from the UAV's point lidars.
  /// This sensor data is then processed to help the UAV hover at a consistent height above the floor.
  private: void ControlLoop(
    const sensor_msgs::LaserScanConstPtr & topLidarMsg,
    const sensor_msgs::LaserScanConstPtr & bottomLidarMsg
  );

  /// \brief Called when the UAV is in the kReset state.
  /// Uses the UAV's bottom point lidar data to determine the appropriate height to reset to.
  /// Once the appropriate height has been determined,
  /// the controller changes to the kMaintain state.
  private: void ResetAltitude(
    const sensor_msgs::LaserScanConstPtr & topLidarMsg,
    const sensor_msgs::LaserScanConstPtr & bottomLidarMsg
  );

  /// \brief Called when the UAV is in the kMaintain state.
  /// Uses the UAV's bottom point lidar to keep the UAV flying at a stable height above the floor.
  /// If the point lidar data indicates that the UAV is closer than minLidarDist to the
  /// ceiling or floor, the controller changes to the kReset state.
  private: void MaintainAltitude(
    const sensor_msgs::LaserScanConstPtr & topLidarMsg,
    const sensor_msgs::LaserScanConstPtr & bottomLidarMsg
  );

  /// \brief Publishes a std_msgs::bool msg with the data set to true if the controller is in
  /// the kReset state. Msg data is set to false otherwise.
  /// \param[in] time The time the message was generated
  private: void PublishResetStateStatus(const ros::Time time);

  /// \brief Sets the controller to the kMaintain state.
  /// \param[in] time The time at which the state was changed to kMaintain
  private: void ChangeToMaintain(ros::Time time);

  /// \brief Instantiates a TwistStamped msg, with all twist fields set to 0
  /// \param[in] time The time for the header.stamp field of the msg
  private: geometry_msgs::TwistStamped MakeTwistStamped(const ros::Time time) const;

  /// \brief ROS nodehandle. Used for subscribing to lidar data, along with publishing
  /// command velocities for the UAV.
  private: ros::NodeHandle nh;

  /// \brief Subscriber for the UAV's upward-facing point lidar.
  private: message_filters::Subscriber<sensor_msgs::LaserScan> topLidarSub;

  /// \brief Subscriber for the UAV's downward-facing point lidar.
  private: message_filters::Subscriber<sensor_msgs::LaserScan> bottomLidarSub;

  /// \brief Time synchronizer; allows for data processing of the UAV's point lidar
  /// sensors from similar points in time.
  // private: message_filters::TimeSynchronizer
  private: message_filters::Synchronizer<ApproximationSyncPolicy> timeSync;

  /// \brief ROS publisher. Publishes command velocities that keep the UAV's
  /// height above the floor consistent.
  private: ros::Publisher hoverVelPub;

  /// \brief ROS publisher. Publishes whether the controller is in the kReset state or not.
  private: ros::Publisher resetStateStatusPub;

  /// \brief The current state of the flight controller.
  private: HoverState state;

  /// \brief The upper bound of the height range the UAV must meet when flying.
  /// This is defined as hoverHeight + hoverTolerance.
  private: double upperHoverBound;

  /// \brief The lower bound of the height range the UAV must meet when flying.
  /// This is defined as hoverHeight - hoverTolerance.
  private: double lowerHoverBound;

  /// \brief ROS nodehandle used for setting parameters
  private: ros::NodeHandle private_nh;

  /// \brief ROS Param. The vertical command velocity to be applied when in the reset state.
  /// (default = 0.4)
  private: double resetVerticalVel;

  /// \brief ROS Param. The vertical command velocity to be applied when in the maintain state.
  /// (default = 0.1)
  private: double maintainVerticalVel;

  /// \brief ROS Param. The height above the floor the UAV should fly.
  /// (default = .55)
  private: double hoverHeight;

  /// \brief ROS Param. How strictly the UAV should match the hoverHeight param when flying.
  /// (default = .02)
  private: double hoverTolerance;

  /// \brief ROS Param. How far away the UAV should stay from the ceiling or floor.
  /// If the top or bottom point lidar ranges read less than this value,
  /// the controller will switch the the kReset state.
  /// (default = .25)
  private: double minLidarDist;
};

#endif
