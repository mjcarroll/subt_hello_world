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

#include "uav_navigation/FlightController.hh"
#include "uav_nav_msgs/ResetStatus.h"

FlightController::FlightController(
  const std::string & topLidarTopic,
  const std::string & bottomLidarTopic,
  const std::string & cmdVelTopic,
  const std::string & resetStatusTopic
) :
topLidarSub(this->nh,topLidarTopic, 1),
bottomLidarSub(this->nh, bottomLidarTopic, 1),
timeSync(ApproximationSyncPolicy(20), this->topLidarSub, this->bottomLidarSub),
state(kReset),
private_nh("~")
{
  ROS_INFO("controller reset state");

  this->private_nh.param("resetVerticalVel", this->resetVerticalVel, 0.4);
  this->private_nh.param("maintainVerticalVel", this->maintainVerticalVel, 0.1);
  this->private_nh.param("hoverHeight", this->hoverHeight, 0.55);
  this->private_nh.param("hoverTolerance", this->hoverTolerance, 0.02);
  this->private_nh.param("minLidarDist", this->minLidarDist, 0.25);

  this->upperHoverBound = this->hoverHeight + this->hoverTolerance;
  this->lowerHoverBound = this->hoverHeight - this->hoverTolerance;

  this->hoverVelPub = this->nh.advertise<geometry_msgs::TwistStamped>(cmdVelTopic, 1);
  this->resetStateStatusPub = this->nh.advertise<uav_nav_msgs::ResetStatus>(resetStatusTopic, 1);
  this->timeSync.registerCallback(
    boost::bind(&FlightController::ControlLoop, this, _1, _2)
    );
}

void FlightController::ControlLoop(
  const sensor_msgs::LaserScanConstPtr & topLidarMsg,
  const sensor_msgs::LaserScanConstPtr & bottomLidarMsg
)
{
  switch (this->state)
  {
    case kReset:
      this->ResetAltitude(topLidarMsg, bottomLidarMsg);
      break;
    case kMaintain:
      this->MaintainAltitude(topLidarMsg, bottomLidarMsg);
      break;
    default:
      ROS_ERROR("Invalid state in FlightController::ControlLoop");
      ros::shutdown();
  }
}

void FlightController::ResetAltitude(
  const sensor_msgs::LaserScanConstPtr & topLidarMsg,
  const sensor_msgs::LaserScanConstPtr & bottomLidarMsg
)
{
  auto time = ros::Time::now();

  if ((bottomLidarMsg->ranges[0] >= this->lowerHoverBound) &&
    (bottomLidarMsg->ranges[0] <= this->upperHoverBound))
  {
    this->ChangeToMaintain(time);
    return;
  }

  geometry_msgs::TwistStamped cmdVel = this->MakeTwistStamped(time);
  cmdVel.twist.linear.z = bottomLidarMsg->ranges[0] < this->lowerHoverBound ? this->resetVerticalVel : -this->resetVerticalVel;

  this->hoverVelPub.publish(cmdVel);
  this->PublishResetStateStatus(time);
}

void FlightController::MaintainAltitude(
  const sensor_msgs::LaserScanConstPtr & topLidarMsg,
  const sensor_msgs::LaserScanConstPtr & bottomLidarMsg
)
{
  auto time = ros::Time::now();

  if ((topLidarMsg->ranges[0] <= this->minLidarDist) ||
    (bottomLidarMsg->ranges[0] <= this->minLidarDist))
  {
    this->state = kReset;
    ROS_INFO("controller reset state");
    this->PublishResetStateStatus(time);
    return;
  }

  geometry_msgs::TwistStamped cmdVel = this->MakeTwistStamped(time);
  if (bottomLidarMsg->ranges[0] < this->lowerHoverBound)
  {
    cmdVel.twist.linear.z = this->maintainVerticalVel;
  }
  else if (bottomLidarMsg->ranges[0] > this->upperHoverBound)
  {
    cmdVel.twist.linear.z = -this->maintainVerticalVel;
  }

  this->hoverVelPub.publish(cmdVel);
  this->PublishResetStateStatus(time);
}

void FlightController::PublishResetStateStatus(const ros::Time time)
{
  uav_nav_msgs::ResetStatus msg;
  msg.header.stamp = time;
  msg.inResetState = this->state == kReset ? true : false;
  this->resetStateStatusPub.publish(msg);
}

void FlightController::ChangeToMaintain(ros::Time time)
{
  this->state = kMaintain;
  ROS_INFO("controller maintain state");
  this->PublishResetStateStatus(time);
}

geometry_msgs::TwistStamped FlightController::MakeTwistStamped(const ros::Time time) const
{
  geometry_msgs::TwistStamped msg;
  msg.header.stamp = time;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.0;
  msg.twist.linear.x = 0.0;
  msg.twist.linear.y = 0.0;
  msg.twist.linear.z = 0.0;

  return msg;
}
