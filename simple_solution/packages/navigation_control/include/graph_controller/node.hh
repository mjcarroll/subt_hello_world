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

#ifndef GRAPH_CONTROLLER__NODE_HH_
#define GRAPH_CONTROLLER__NODE_HH_

#include <string>
#include <unordered_set>

#include <geometry_msgs/Point.h>

namespace graph_controller {

struct Node
{
  // Representation of the state of this node
  enum class State
  {
    // Node has been seen but not visited
    kUnexplored,
    // Node has been seen and visited
    kExplored,
    // Node has been seen and cannot be visited
    kInaccessible,
  };

  // Name for disambiguating node (robot name)
  std::string name;

  // Unique integer id for node
  std::string id;

  // Unique integer id for node
  int vizualization_id;

  // Node location
  geometry_msgs::Point location;

  // Connection to node neighbors
  std::vector<Node*> neighbors;

  // Connection to node neighbors by id
  std::unordered_set<std::string> neighbors_by_id;

  // State of the node
  State state;

  // Determine if node is accessible
  // which means it should be considered by graph algorithms
  bool isAccessible () const { return state != State::kInaccessible; };

  // Determine if node is unexplored
  bool isUnexplored () const { return state == State::kUnexplored; };

};

}  // namespace graph_controller

#endif  // GRAPH_CONTROLLER__NODE_HH_
