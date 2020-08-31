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

#include <frontier_finder/costmap_client.hh>

#include <functional>
#include <mutex>
#include <string>

using namespace frontier_finder;

// static translation table to speed things up
std::array<unsigned char, 256> init_translation_table();
static const std::array<unsigned char, 256> kCostTranslationTable_ =
    init_translation_table();

CostmapClient::CostmapClient(const tf2_ros::Buffer* tf_buffer)
: tf_buffer_(tf_buffer)
{

  ros::NodeHandle nh;
  std::string nodeName = ros::this_node::getName();
  std::string robotName;
  nh.param<std::string>(nodeName + "/name", robotName, "X1");
  nh.param<std::string>(nodeName + "/robot_base_frame", this->robot_base_frame_, robotName);
  nh.param<double>(nodeName + "/transform_tolerance", this->transform_tolerance_, 0.3);

  /* initialize costmap */
  std::string costmap_topic("/" + robotName + "/move_base/global_costmap/costmap");
  std::string costmap_updates_topic("/" + robotName +  "/move_base/global_costmap/costmap_updates");
  costmap_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>(
      costmap_topic, 1000,
      [this](const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        updateFullMap(msg);
      });
  ROS_INFO("Waiting for costmap to become available, topic: %s",
           costmap_topic.c_str());
  auto costmap_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(
      costmap_topic, nh);
  ROS_INFO("Costmap is available");
  updateFullMap(costmap_msg);

  /* subscribe to map updates */
  costmap_updates_sub_ =
      nh.subscribe<map_msgs::OccupancyGridUpdate>(
          costmap_updates_topic, 1000,
          [this](const map_msgs::OccupancyGridUpdate::ConstPtr& msg) {
            updatePartialMap(msg);
          });

  ros::Time last_error = ros::Time::now();
  bool canTransform = false;

  // Iterate until the transform from the global frame to the robot base frame becomes available.
  while (ros::ok() &&  !canTransform)
  {
    ros::spinOnce();
    try
    {
      tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, ros::Time(),
                                  ros::Duration(0.1));
      canTransform = true;
    }
    catch(const tf2::TransformException & ex)
    {
      ROS_WARN("tf2::TransformException in CostmapClient::CostmapClient: %s", ex.what());
    }

    if (last_error + ros::Duration(5.0) < ros::Time::now()) {
      ROS_WARN(
          "Timed out waiting for transform from %s to %s to become available "
          "before subscribing to costmap",
          robot_base_frame_.c_str(), global_frame_.c_str());
      last_error = ros::Time::now();
    }
  }
}

void CostmapClient::updateFullMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  global_frame_ = msg->header.frame_id;

  unsigned int size_in_cells_x = msg->info.width;
  unsigned int size_in_cells_y = msg->info.height;
  double resolution = msg->info.resolution;
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;

  ROS_DEBUG("received full new map, resizing to: %d, %d", size_in_cells_x,
            size_in_cells_y);
  costmap_.resizeMap(size_in_cells_x, size_in_cells_y, resolution, origin_x,
                     origin_y);

  // lock as we are accessing raw underlying map
  auto* mutex = costmap_.getMutex();
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  // fill map with data
  unsigned char* costmap_data = costmap_.getCharMap();
  size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
  ROS_DEBUG("full map update, %lu values", costmap_size);
  for (size_t i = 0; i < costmap_size && i < msg->data.size(); ++i) {
    unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
    costmap_data[i] = kCostTranslationTable_[cell_cost];
  }
  ROS_DEBUG("map updated, written %lu values", costmap_size);
}

void CostmapClient::updatePartialMap(
    const map_msgs::OccupancyGridUpdate::ConstPtr& msg)
{
  //ROS_DEBUG("received partial map update");
  global_frame_ = msg->header.frame_id;

  if (msg->x < 0 || msg->y < 0) {
    ROS_ERROR("negative coordinates, invalid update. x: %d, y: %d", msg->x,
              msg->y);
    return;
  }

  size_t x0 = static_cast<size_t>(msg->x);
  size_t y0 = static_cast<size_t>(msg->y);
  size_t xn = msg->width + x0;
  size_t yn = msg->height + y0;

  // lock as we are accessing raw underlying map
  auto* mutex = costmap_.getMutex();
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  size_t costmap_xn = costmap_.getSizeInCellsX();
  size_t costmap_yn = costmap_.getSizeInCellsY();

  if (xn > costmap_xn || x0 > costmap_xn || yn > costmap_yn ||
      y0 > costmap_yn) {
    ROS_WARN("received update doesn't fully fit into existing map, "
             "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
             "map is: [0, %lu], [0, %lu]",
             x0, xn, y0, yn, costmap_xn, costmap_yn);
  }

  // update map with data
  unsigned char* costmap_data = costmap_.getCharMap();
  size_t i = 0;
  for (size_t y = y0; y < yn && y < costmap_yn; ++y) {
    for (size_t x = x0; x < xn && x < costmap_xn; ++x) {
      size_t idx = costmap_.getIndex(x, y);
      unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
      costmap_data[idx] = kCostTranslationTable_[cell_cost];
      ++i;
    }
  }
}

geometry_msgs::PoseStamped CostmapClient::getRobotPose() const
{

  ros::Time current_time =
      ros::Time::now();  // save time for checking tf delay later
  geometry_msgs::PoseStamped global_pose, robot_pose;

  // get the global pose of the robot
  try {

    geometry_msgs::TransformStamped trans =
        tf_buffer_->lookupTransform(
            global_frame_, robot_base_frame_, ros::Time(), ros::Duration(1.0));
    tf2::doTransform(robot_pose, global_pose, trans);

    //tf_buffer_->transformPose(global_frame_, robot_pose, global_pose);
  } catch (tf2::LookupException& ex) {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot "
                            "pose: %s\n",
                       ex.what());
    return {};
  } catch (tf2::ConnectivityException& ex) {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n",
                       ex.what());
    return {};
  } catch (tf2::ExtrapolationException& ex) {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n",
                       ex.what());
    return {};
  }

  global_pose.header.frame_id = global_frame_;
  return global_pose;
}

std::array<unsigned char, 256> init_translation_table()
{
  std::array<unsigned char, 256> cost_translation_table;

  // lineary mapped from [0..100] to [0..255]
  for (size_t i = 0; i < 256; ++i) {
    cost_translation_table[i] =
        static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
  }

  // special values:
  cost_translation_table[0] = 0;      // NO obstacle
  cost_translation_table[99] = 253;   // INSCRIBED obstacle
  cost_translation_table[100] = 254;  // LETHAL obstacle
  cost_translation_table[static_cast<unsigned char>(-1)] = 255;  // UNKNOWN

  return cost_translation_table;
}

costmap_2d::Costmap2D* CostmapClient::getCostmap()
{
  return &costmap_;
}

const std::string& CostmapClient::getGlobalFrameID() const {
  return global_frame_;
}
