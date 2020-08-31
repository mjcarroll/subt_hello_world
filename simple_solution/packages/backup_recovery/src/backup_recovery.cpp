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

#include <backup_recovery/backup_recovery.hh>

#include <geometry_msgs/Twist.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/utils.h>

namespace backup_recovery
{

void BackupRecovery::initialize(std::string name,
                                tf2_ros::Buffer* /*tf*/,
                                costmap_2d::Costmap2DROS* /*global_costmap*/,
                                costmap_2d::Costmap2DROS* local_costmap)
{
  if (this->initialized_)
  {
    return;
  }

  ros::NodeHandle backup_nh("~/" + name);
  backup_nh.param<float>("backwards_vel", this->backwards_vel_, -0.3);
  backup_nh.param<float>("backup_dist", this->backup_dist_, 0.5);
  backup_nh.param<float>("frequency", this->frequency_, 20.0);

  this->local_costmap_ = local_costmap;
  this->world_model_ = std::make_shared<base_local_planner::CostmapModel>(*local_costmap_->getCostmap());

  this->initialized_ = true;
}

void BackupRecovery::runBehavior()
{
  if (!this->initialized_)
  {
    ROS_ERROR("This object must be initialized before BackupRecovery::runBehavior() is called.");
    return;
  }
  else if (!this->local_costmap_)
  {
    ROS_ERROR("Local costmap is NULL in BackupRecovery::runBehavior(). Doing nothing.");
    return;
  }

  ROS_WARN("Running backup recovery behavior...");

  ros::Rate r(this->frequency_);
  ros::NodeHandle nh;
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  geometry_msgs::PoseStamped curr_pose;
  this->local_costmap_->getRobotPose(curr_pose);

  if (this->potentialCollision(curr_pose))
  {
    return;
  }

  double x_orig = curr_pose.pose.position.x;
  double y_orig = curr_pose.pose.position.y;

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = this->backwards_vel_;
  cmd_vel.linear.y = 0;
  cmd_vel.linear.z = 0;
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = 0;
  vel_pub.publish(cmd_vel);

  while (nh.ok())
  {
    this->local_costmap_->getRobotPose(curr_pose);
    double x_new = curr_pose.pose.position.x;
    double y_new = curr_pose.pose.position.y;
    double x_diff = x_new - x_orig;
    double y_diff = y_new - y_orig;
    double dist = sqrt((x_diff * x_diff) + (y_diff * y_diff));
    if (dist >= this->backup_dist_)
    {
      cmd_vel.linear.x = 0;
      vel_pub.publish(cmd_vel);
      ROS_WARN("Backup recovery behavior has been completed.");
      return;
    }

    if (this->potentialCollision(curr_pose))
    {
      return;
    }

    r.sleep();
  }
}

bool BackupRecovery::potentialCollision(const geometry_msgs::PoseStamped & pose)
{
  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double theta = tf2::getYaw(pose.pose.orientation);

  double cost = this->world_model_->footprintCost(x, y, theta,
                                                  this->local_costmap_->getRobotFootprint());
  if (cost < 0.0)
  {
    ROS_ERROR("Potential collision; robot cannot backup anymore.");
    return true;
  }

  return false;
}
}; // namespace backup_recovery

// register this class as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(backup_recovery::BackupRecovery, nav_core::RecoveryBehavior)
