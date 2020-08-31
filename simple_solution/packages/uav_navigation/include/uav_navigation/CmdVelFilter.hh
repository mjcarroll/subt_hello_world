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

#ifndef UAV_NAVIGATION_CMD_VEL_FILTER_H_
#define UAV_NAVIGATION_CMD_VEL_FILTER_H_

#include "ros/ros.h"
#include "uav_nav_msgs/ResetStatus.h"
#include "geometry_msgs/TwistStamped.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"

/// \brief Approximate time policy that allows for synchronization of nearly-exact
/// time stamped msgs
typedef message_filters::sync_policies::ApproximateTime<uav_nav_msgs::ResetStatus,
  geometry_msgs::TwistStamped, geometry_msgs::TwistStamped> ApproximateCmdVelPolicy;

/// \class CmdVelFilter CmdVelFilter.hh
/// \brief Class for deciding how to apply cmdVels from move_base and
/// flightController to the UAV
class CmdVelFilter
{
  /// \brief Constructor
  /// \param[in] resetStatusTopic The topic for uav_nav_msgs::ResetStatus msgs
  /// \param[in] controllerCmdVelTopic The topic for cmdVel msgs that come from the
  /// flight controller (geometry_msgs::TwistStamped)
  /// \param[in] moveBaseCmdVelTopic The topic for the cmdVel msgs published by
  /// move_base (geometry_msgs::Twist)
  /// \param[in] moveBaseCmdVelTopicStamped The topic for modified time-stamped cmdVel
  /// msgs published by move_base (geometry_msgs::TwistStamped)
  /// \param[in] robotCmdVelTopic The topic where filtered cmdVel msgs are published
  /// (these are the final cmdVels that control the robot)
  public: CmdVelFilter(
    const std::string & resetStatusTopic = "reset_state_status",
    const std::string & controllerCmdVelTopic = "controller_cmd_vel",
    const std::string & moveBaseCmdVelTopic = "move_base_cmd_vel",
    const std::string & moveBaseCmdVelStampedTopic = "move_base_cmd_vel_stamped",
    const std::string & robotCmdVelTopic = "cmd_vel"
  );

  /// \brief Approximate time CB. Called when it is time to filter the next cmdVel
  /// to the robot
  /// \param[in] resetStateStatus The uav_nav_msgs::ResetStatus msg that indicates
  /// whether the flight controller is in the reset state or not
  /// \param[in] controllerCmdVel The cmdVel from the flight controller that
  /// corresponds to the state indicated by resetStateStatus
  /// (geometry_msgs::TwistStamped)
  /// \param[in] moveBaseCmdVel The most recent cmdVel from move_base that has
  /// been modified to include a time stamp (geometry_msgs::TwistStamped)
  private: void FilterCmdVels(
    const uav_nav_msgs::ResetStatusConstPtr & resetStateStatus,
    const geometry_msgs::TwistStampedConstPtr & controllerCmdVel,
    const geometry_msgs::TwistStampedConstPtr & moveBaseCmdVel
  );

  /// \brief Makes a geometry_msgs::TwistStamped msg from a geometry_msgs::Twist msg
  /// \param[in] msg The msg used to fill the twist fields for the new
  /// geometry_msgs::TwistStamped msg
  private: void MakeTwistStamped(const geometry_msgs::TwistConstPtr & msg);

  /// \brief ROS nodehandle, used for publishing and subscribing cmdVels
  private: ros::NodeHandle nh;

  /// \brief Subscriber for the original cmdVels (geometry_msgs::Twist) that come
  /// from move_base
  private: ros::Subscriber originalMoveBaseCmdVelSub;

  /// \brief Publisher for the modified cmdVel from move_base that has been time stamped
  /// (geometry_msgs::TwistStamped)
  private: ros::Publisher moveBaseStampedPub;

  /// \brief Publisher for the final filtered cmdVel that is sent to the robot
  /// (geometry_msgs::Twist)
  private: ros::Publisher filteredCmdVelPub;

  /// \brief Subscriber for the reset state status msgs from the flight controller
  /// (uav_nav_msgs::ResetStatus)
  private: message_filters::Subscriber<uav_nav_msgs::ResetStatus> resetStateSub;

  /// \brief Subscriber for the cmdVels that come from the flight controller
  /// (geometry_msgs::TwistStamped)
  private: message_filters::Subscriber<geometry_msgs::TwistStamped> controllerCmdVelSub;

  /// \brief Subscriber for the modified time stamped cmdVels that were originally
  /// published by move_base (geometry_msgs::TwistStamped)
  private: message_filters::Subscriber<geometry_msgs::TwistStamped> stampedMoveBaseSub;

  /// \brief Approximate time synchronizer for the stamped cmdVels from the flight
  /// controller and move_base, along with the controller reset status msgs
  private: message_filters::Synchronizer<ApproximateCmdVelPolicy> timeSync;
};

#endif
