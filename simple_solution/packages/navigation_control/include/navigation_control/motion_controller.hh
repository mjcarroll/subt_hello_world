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

#ifndef NAVIGATION_CONTROL__MOTION_CONTROLLER_HH_
#define NAVIGATION_CONTROL__MOTION_CONTROLLER_HH_

#include <functional>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace navigation_control {

class MotionController
{
public:
  MotionController(std::function<void(bool)> callback, int timeout);

  void MoveToPoint(const geometry_msgs::Point & point, const std::string & frame);

private:
  /// \brief The callback that is triggered when the move_base action finishes
  void DoneCB(
    const actionlib::SimpleClientGoalState & state,
    const move_base_msgs::MoveBaseResultConstPtr & result);

  void activeCB();

  /// \brief Handle to create timer
  ros::NodeHandle nh_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mb_client_;

  /// \brief The callback from a client that is triggered to let the client know
  /// a motion command has been completed.
  /// This function must be of void return type and takes in a bool as a parameter.
  /// The bool represents whether the motion command was completed successfully or not.
  std::function<void(bool)> external_cb_;

  /// \brief Timer used to cancel the action
  ros::Timer cancel_action_oneshot_timer_;

  /// \brief Time (in seconds) for action timeout.
  int timeout_;
};

}  // namespace navigation_control

#endif  // NAVIGATION_CONTROL__MOTION_CONTROLLER_HH_
