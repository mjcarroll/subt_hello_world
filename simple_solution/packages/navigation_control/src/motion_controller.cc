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

#include <navigation_control/motion_controller.hh>

#include <ros/ros.h>

using namespace navigation_control;

MotionController::MotionController(std::function<void(bool)> callback, int timeout) :
  mb_client_("move_base", true),
  external_cb_(callback),
  timeout_(timeout)
{
  this->mb_client_.waitForServer();
  ROS_DEBUG("Motion up and running");
}

void MotionController::MoveToPoint(const geometry_msgs::Point & point, const std::string & frame)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = frame;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position = point;
  goal.target_pose.pose.orientation.w = 1.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;

  this->mb_client_.sendGoal(
    goal,
    boost::bind(&MotionController::DoneCB, this, _1, _2),
    boost::bind(&MotionController::activeCB, this)
  );
}

void MotionController::DoneCB(
  const actionlib::SimpleClientGoalState & state,
  const move_base_msgs::MoveBaseResultConstPtr & result
)
{
  this->cancel_action_oneshot_timer_.stop();
  if (state.state_ == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
  {
    this->external_cb_(true);
  }
  else
  {
    ROS_WARN_STREAM("Goal was not reached successfully; goal state is " << state.toString());
    this->external_cb_(false);
  }
}

void MotionController::activeCB()
{
  this->cancel_action_oneshot_timer_ = this->nh_.createTimer(
    ros::Duration(this->timeout_, 0),
    [this](const ros::TimerEvent&) { this->mb_client_.cancelGoal(); },
    true);
}
