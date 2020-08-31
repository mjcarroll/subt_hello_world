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

#ifndef FRONTIER_FINDER__FRONTIER_FINDER_HH_
#define FRONTIER_FINDER__FRONTIER_FINDER_HH_

#include <utility>
#include <string>
#include <vector>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <frontier_finder/costmap_client.hh>

namespace frontier_finder 
{
// (x,y) coordinates of a point in the map
typedef std::pair<int,int> point_t;

/// \class FrontierFinder frontier_finder.hh
/// \brief Class for finding "frontiers" (or possible navigable points)
/// near the robot's current pose
class FrontierFinder
{
public:
  /// \brief Construtor
  /// \param[in] frontier_dist How far away frontiers should be generated from the robot
  /// \param[in] min_frontier_size  Minimum size of the frontier (in cells)
  FrontierFinder(
    const float frontier_dist = 5.0,
    const float min_frontier_size = 1.0,
    const float max_frontier_size = 1000.0,
    const float plan_tolerance = 0.5
    );

  /// \brief Finds a list of valid frontiers near the robot's current pose
  /// \param[in] frame Name of the frame where frontiers should be.
  /// \returns A vector of points where the locations are in
  std::vector<geometry_msgs::Point> findFrontiers(const std::string & frame);

private:

  /// \brief Determines if the given location is obstacle free.
  /// \param[in] loc The location that represents the location.
  bool isObstacleFree(const point_t& point);

  /// \brief Transform objects, used for getting the robot's current pose and
  // for transforming the frontier locations into this->location_frame_
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  /// \brief The costmap client that keeps track of obstacles from sensor data
  CostmapClient costmap_client_;

  /// \brief Get cells in an octan from radius.
  std::vector<point_t> getOctantPoints(const int &  radius);

  /// \brief Get cells in a circle from an octant
  std::vector<point_t> getCirclePointsFromOctant(const std::vector<point_t>& octant);

  /// \brief Get the pose of the robot in map (cells)
  point_t getRobotPoseInMap();

  /// \brief Gets the cell coordinates in the map from a point in the circle vector
  /// with center in a given point
  point_t getCirclePointWithCenter(const int & circle_index, const point_t & center );

  /// \brief Gets the first occupied point in a circle with a given center.
  int getFirstOccupiedPointIndex(const point_t & center );

  /// \brief Transform frontiers from points in map to points in world.
  std::vector<geometry_msgs::Point> transformFrontiers(
                                const std::vector<point_t>& frontiers,
                                std::string frame_id);

  /// \brief Get a plan to a point.
  bool getPlan(const geometry_msgs::PoseStamped &start,
               const geometry_msgs::PoseStamped &goal,
               const float & tolerance,
               std::vector<geometry_msgs::PoseStamped> &plan);

  /// \brief Given a vector of frontiers it returns a vector with frontiers that are accesible from
  /// robot's current pose.
  std::vector<point_t> getAccesibleFrontiers(
                                const std::vector<point_t> & frontiers);

  /// \brief Publishes a series of markers based on a vector of points
  void publishPoints(std::vector<geometry_msgs::Point> frontiers, const std::string & ns, const std::string & frame,
                        const ros::Publisher & publisher, double scale, float r, float g, float b);

  /// \brief Deletes a series of markers
  void deletePoints(const std::string &ns, const std::string & frame, const ros::Publisher & publisher);

  /// \brief Checks that there is an obstacle free line between robot position
  /// and the frontier.
  bool isFrontierAccesible(const point_t & point);

  /// \brief Uses the local costmap to perform collision checking
  base_local_planner::CostmapModel world_model_;

  /// \brief Minimum size of a frontier (in cells)
  const int min_frontier_size_;

  /// \brief Maximum size of a frontier (in cells)
  const int max_frontier_size_;

  /// \brief Points in the circle of radius r
  std::vector<point_t> points_in_circle_;

  /// \brief Publishers for markers
  ros::Publisher frontier_pub_;
  ros::Publisher frontier_candidates_pub_;
  ros::Publisher circle_pub_;

  /// \brief Is debug mode on?
  bool debug_{false};

  /// \brief Frontier accesible plan tolerance
  float plan_tolerance_;

  /// \brief Distance (in meters) to the frontiers
  float frontier_dist_;
};
}  // namespace frontier_finder


#endif  // FRONTIER_FINDER__FRONTIER_FINDER_HH_
