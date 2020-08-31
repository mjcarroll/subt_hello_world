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

#include <graph_controller/serializer.hh>

#include <unordered_set>

#include <ros/ros.h>
#include <navigation_control/protobuf/graph.pb.h>
#include <geometry_msgs/Point.h>

namespace graph_controller {

std::string Serializer::SerializeGraph(const std::list<Node> & graph) const
{
  navigation_control::msgs::SerializableGraph serie_graph;
  for (std::list<Node>::const_iterator it = graph.begin(); it != graph.end(); ++it)
  {
    const Node curr_node = *it;
    navigation_control::msgs::SerializableNode* temp = serie_graph.add_nodes();
    temp->set_name(static_cast<std::string>(curr_node.name));
    temp->set_id(static_cast<std::string>(curr_node.id));
    temp->set_loc_x(static_cast<float>(curr_node.location.x));
    temp->set_loc_y(static_cast<float>(curr_node.location.y));
    temp->set_loc_z(static_cast<float>(curr_node.location.z));
    for (std::unordered_set<std::string>::const_iterator it = curr_node.neighbors_by_id.begin(); it != curr_node.neighbors_by_id.end(); ++it)
    {
      std::string* neighbor_id = temp->add_neighbors_by_id();
      *neighbor_id = *it;
    }
    temp->set_state(static_cast<uint32_t>(curr_node.state));
  }
  std::string result;
  if (!serie_graph.SerializeToString(&result))
  {
    ROS_ERROR_STREAM("navigation_control::Serializer(): Error serializing message\n" << serie_graph.DebugString());
  }
  return result;
}

std::list<Node> Serializer::DeSerializeGraph(const std::string & serialized_graph) const
{
  navigation_control::msgs::SerializableGraph serie_graph;
  if (!serie_graph.ParseFromString(serialized_graph))
  {
    ROS_ERROR_STREAM("navigation_control::Serializer(): Error serializing message\n" << serie_graph.DebugString());
  }
  std::list<Node> result;
  for (auto node_index = 0; node_index < serie_graph.nodes_size(); ++node_index)
  {
    const navigation_control::msgs::SerializableNode temp = serie_graph.nodes(node_index);
    Node temp_node;
    temp_node.name = temp.name();
    temp_node.id = temp.id();
    geometry_msgs::Point location;
    location.x = temp.loc_x();
    location.y = temp.loc_y();
    location.z = temp.loc_z();
    temp_node.location = location;
    std::unordered_set<std::string> neighbors_by_id;
    for (auto neighbor_index = 0; neighbor_index < temp.neighbors_by_id_size(); ++neighbor_index)
    {
      std::string neighbor_id = temp.neighbors_by_id(neighbor_index);
      neighbors_by_id.insert(neighbor_id);
    }
    temp_node.neighbors_by_id = neighbors_by_id;
    temp_node.state = static_cast<graph_controller::Node::State>(temp.state());
    result.push_back(temp_node);
  }
  return result;
}

}  // namespace graph_controller
