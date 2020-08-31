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

#ifndef BACKUP_RECOVERY__BACKUP_RECOVERY_HH_
#define BACKUP_RECOVERY__BACKUP_RECOVERY_HH_

#include <memory>
#include <string>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/recovery_behavior.h>
#include <tf2_ros/buffer.h>

namespace backup_recovery
{
/// \brief class BackupRecovery backup_recovery.hh
/// \brief A recovery behavior that backs the robot up in case it is too close to an obstacle
class BackupRecovery : public nav_core::RecoveryBehavior
{
public:
  /// \brief Initialization function for the recovery behavior
  /// \param[in] name The name of the recovery behavior
  /// \param[in] tf (unused)
  /// \param[in] global_costmap (unused)
  /// \param[in] local_costmap A pointer to the local_costmap used by the navigation stack
  void initialize(std::string name,
                  tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* global_costmap,
                  costmap_2d::Costmap2DROS* local_costmap);

  /// \brief Run the backup recovery behavior
  void runBehavior();

private:
  /// \brief Checks to see if the robot's current pose is a possible collision
  /// \param[in] pose The current pose
  /// \return true if there's a possible collision, false otherwise
  bool potentialCollision(const geometry_msgs::PoseStamped & pose);

  /// \param backwards_vel_ The velocity to apply to the robot when moving it backwards
  /// Default = -0.3
  float backwards_vel_;

  /// \param backup_dist_ How far the robot should back up
  /// Default = 0.5
  float backup_dist_;

  /// \param frequency_ The rate (in Hz) at which the robot's backup progress is checked
  /// Default = 20.0
  float frequency_;

  /// \brief Indicates when the class's private data members have been initialized
  bool initialized_ {false};

  /// \brief The local costmap from move_base
  costmap_2d::Costmap2DROS* local_costmap_ {nullptr};

  /// \brief Uses the local costmap to perform collision checking
  std::shared_ptr<base_local_planner::CostmapModel> world_model_ {nullptr};
};
}  // namespace backup_recovery

#endif  // BACKUP_RECOVERY__BACKUP_RECOVERY_HH_
