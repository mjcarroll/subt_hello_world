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

#ifndef GRAPH_CONTROLLER__GRAPH_HH_
#define GRAPH_CONTROLLER__GRAPH_HH_

#include <list>
#include <mutex>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <graph_controller/node.hh>

namespace graph_controller {

using Path = std::list<Node*>;

class Graph
{
public:
  Graph();

  Node* AddNode(const geometry_msgs::Point &loc,
          const Node::State &_state=Node::State::kUnexplored,
          const std::vector<Node*> _neighbors={});

  Path ComputePath(const Node *_begin, const Node *_end);

  double CalculateDistance(const geometry_msgs::Point & start,
                    const geometry_msgs::Point & goal) const;

  int MinScore(const std::vector<std::string> & seen, const std::unordered_map<std::string, double> & f_score) const;

  void DrawNode(const geometry_msgs::Point & point, float r, float g, float b, int id);

  void DrawPath(const geometry_msgs::Point & point1, const geometry_msgs::Point & point2, float r, float g, float b, int id);

  std::list<Node*> GetClosestUnexploredNodes(const Node *_begin);

  void UpdateNodeState(const Node* node, const Node::State &_state);

  void MergeGraph(const std::list<Node> & mergin_graph);

  std::list<Node> GetGraph() const;

  float ComputeOrientation(const Node *_from, const Node *_current, const Node *_destiny);

  void SetGraph(std::list<Node> nodes);

  void SetRobotName(const std::string & robot_name);

  void PrintGraph() const;

private:

  void MakeNeighbors(Node* node1, Node* node2);

  Node* addNode(const geometry_msgs::Point &loc,
          const Node::State &_state=Node::State::kUnexplored,
          const std::vector<Node*> _neighbors={});

  // Storage structure for graph nodes
  std::list<Node> nodes_;

  // Map of node IDs to a given node
  std::unordered_map<std::string, Node*> nodes_by_id_;

  // ID to give to the next node added to the graph
  uint64_t nextId_ {0};

  // Robot name to be used on the node's keys
  std::string robot_name_;

  // Max distance for each path
  float path_distance_;

  // Node handle to publish rviz markers msgs
  ros::NodeHandle nh_;

  // Parameter for enabling rviz debug visualization
  bool debug_;

  // Id for rviz debugging visualization paths
  int debug_path_id_;

  /// \brief Publisher for rviz visualisation markers
  ros::Publisher debug_pub_;

  // The name of the artifact origin frame
  std::string artifact_origin_frame_;

  /// \brief Minimun distance between nodes to be considered different nodes
  float lock_percentage_;

  /// \brief Mutex to separete merging from using the graph.
  std::mutex graph_mutex_;
};

}  // namespace graph_controller

#endif  // GRAPH_CONTROLLER__GRAPH_HH_
