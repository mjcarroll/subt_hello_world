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

#include "uav_navigation/CmdVelFilter.hh"
#include "geometry_msgs/Twist.h"

CmdVelFilter::CmdVelFilter(
  const std::string & resetStatusTopic,
  const std::string & controllerCmdVelTopic,
  const std::string & moveBaseCmdVelTopic,
  const std::string & moveBaseCmdVelStampedTopic,
  const std::string & robotCmdVelTopic
) :
resetStateSub(this->nh, resetStatusTopic, 1),
controllerCmdVelSub(this->nh, controllerCmdVelTopic, 1),
stampedMoveBaseSub(this->nh, moveBaseCmdVelStampedTopic, 1),
timeSync(ApproximateCmdVelPolicy(10), this->resetStateSub,
  this->controllerCmdVelSub, this->stampedMoveBaseSub)
{
  this->originalMoveBaseCmdVelSub = this->nh.subscribe<geometry_msgs::Twist>(
    moveBaseCmdVelTopic, 1, boost::bind(&CmdVelFilter::MakeTwistStamped, this, _1)
    );
  this->moveBaseStampedPub =
    this->nh.advertise<geometry_msgs::TwistStamped>(moveBaseCmdVelStampedTopic, 1);
  this->filteredCmdVelPub =
    this->nh.advertise<geometry_msgs::Twist>(robotCmdVelTopic, 1);
  this->timeSync.registerCallback(
    boost::bind(&CmdVelFilter::FilterCmdVels, this, _1, _2, _3)
    );
}

void CmdVelFilter::FilterCmdVels(
  const uav_nav_msgs::ResetStatusConstPtr & resetStateStatus,
  const geometry_msgs::TwistStampedConstPtr & controllerCmdVel,
  const geometry_msgs::TwistStampedConstPtr & moveBaseCmdVel
)
{
  geometry_msgs::Twist robotCmdVel;
  robotCmdVel.angular = controllerCmdVel->twist.angular;
  robotCmdVel.linear = controllerCmdVel->twist.linear;

  if (!resetStateStatus->inResetState)
  {
    robotCmdVel.angular.x += moveBaseCmdVel->twist.angular.x;
    robotCmdVel.angular.y += moveBaseCmdVel->twist.angular.y;
    robotCmdVel.angular.z += moveBaseCmdVel->twist.angular.z;
    robotCmdVel.linear.x += moveBaseCmdVel->twist.linear.x;
    robotCmdVel.linear.y += moveBaseCmdVel->twist.linear.y;
    robotCmdVel.linear.z += moveBaseCmdVel->twist.linear.z;
  }

  this->filteredCmdVelPub.publish(robotCmdVel);
}

void CmdVelFilter::MakeTwistStamped(const geometry_msgs::TwistConstPtr & msg)
{
  geometry_msgs::TwistStamped updated_msg;
  updated_msg.header.stamp = ros::Time::now();
  updated_msg.twist.angular = msg->angular;
  updated_msg.twist.linear = msg->linear;

  this->moveBaseStampedPub.publish(updated_msg);
}
