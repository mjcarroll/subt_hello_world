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

#ifndef GRAPH_CONTROLLER__CONTROLLER_HH_
#define GRAPH_CONTROLLER__CONTROLLER_HH_

#include <functional>
#include <string>
#include <utility>

#include <frontier_finder/frontier_finder.hh>

#include <navigation_control/motion_controller.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <graph_controller/graph.hh>
#include <graph_controller/serializer.hh>

namespace graph_controller {

class Controller
{
public:
  /// \brief Constructor
  /// \param[in] robot_name The name of the robot
  /// \param[in] artifact_origin_frame The name of the artifact origin frame
  /// \param[in] extern_bt_func A function that lets outside users know backtracking started early
  Controller(const std::string & robot_name,
             const std::string & artifact_origin_frame,
             std::function<void()> extern_bt_func=nullptr);

  /// \brief Initialize the graph at the start of the cave
  void Initialize();

  /// \brief Set a goal node in the graph to navigate to
  void SetGoal(const Node* _destination);

  /// \brief Find frontiers and add them as connections
  void ComputeFrontiers(Node* _node);

  /// \brief Compute next point in the graph to explore
  Node* ComputeGoal(Node* _from, Node* _current);

  /// \brief Callback with the results of the motion goal
  void MotionCallback(bool successful);

  /// \brief Tells the robot to stop exploring and to start backtracking
  void Backtrack();

  /// \brief Determines if the robot is done backtracking or not
  /// \returns true if the robot is done backtracking, false otherwise
  bool DoneBacktracking() const;

private:
  /// \brief Move to the next node in the path
  void AdvanceDownPath();

  /// \brief Re-generate the path that can be traversed
  void RecomputePath();

  /// \brief Callback that is triggered when graph info from another robot is received
  void OtherGraphCB(const std_msgs::String::ConstPtr & msg);

  /// \brief Timer method that broadcasts the robot's graph information
  void ShareGraph();

  /// \brief ROS nodehandle
  ros::NodeHandle nh_;

  /// \brief Underlying exploration graph controller
  Graph graph_;

  /// \brief Underlying motion controller
  navigation_control::MotionController motion_controller_;

  Serializer serializer_;

  /// \brief The last visited node, for populating neighbors
  Node* lastVisited_ {nullptr};

  /// \brief The current node we are trying to reach
  Node* currentStep_ {nullptr};

  /// \brief Origin node for bracktracking
  Node* artifact_origin_node_;

  /// \brief The path sequence of navigation goals
  Path path_;

  /// \brief Robot name
  const std::string robotName_;

  /// \brief The name of the artifact origin frame
  const std::string artifact_origin_frame_;

  /// \brief Underlying exploration frontier finder
  frontier_finder::FrontierFinder frontiers_finder_;

  /// \brief used to share a robot's graph information
  ros::Publisher graph_publisher_;
  ros::Subscriber graph_subscriber_;
  ros::Timer graph_timer_;

  /// \brief keeps track of whether the robot is backtracking (done exploring) or not
  bool back_track_state_;
  std::function<void()> extern_bt_func_;

  /// \brief Time in seconds to wait before exploring
  float wait_seconds_before_exploring_;

  /// \brief Distance to be from the origin before merging nodes received from comms
  float distance_until_merge_;

  /// \brief First node's x position in the artifat_origin frame
  float starting_node_x_pos_;

  /// \brief First node's y position in the artifat_origin frame
  float starting_node_y_pos_;

  /// \brief Defines a region (circle) around the artifact_origin frame where no new nodes can be added
  /// (except for the starting node)
  float artifact_origin_space_;
};

}  // namespace graph_controller

#endif  // GRAPH_CONTROLLER__CONTROLLER_HH_
