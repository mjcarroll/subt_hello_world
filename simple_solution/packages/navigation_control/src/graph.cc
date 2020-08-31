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

#include <graph_controller/graph.hh>

#include <unordered_set>
#include <string>

#include <visualization_msgs/Marker.h>

namespace graph_controller {

using Path = std::list<Node*>;

Graph::Graph()
{
  std::string nodeName = ros::this_node::getName();
  this->nh_.param<std::string>(nodeName + "/name", this->robot_name_, "X1");
  this->nh_.param<float>(nodeName + "/path_distance", this->path_distance_, 5.0);
  this->nh_.param<float>(nodeName + "/lock_percentage", this->lock_percentage_, 0.7);
  nh_.param<std::string>(nodeName + "/artifact_origin_frame", this->artifact_origin_frame_, "artifact_origin");
  this->nh_.param<bool>(nodeName + "/debug", this->debug_, false);
  if (this->debug_)
  {
    this->debug_pub_ = this->nh_.advertise<visualization_msgs::Marker>("pathing", 0);
    this->debug_path_id_ = 0;
  }
  ROS_DEBUG("Graph element up and running");
}

Node* Graph::AddNode(const geometry_msgs::Point &loc,
        const Node::State &_state,
        const std::vector<Node*> _neighbors)
{
  const std::lock_guard<std::mutex> lock(this->graph_mutex_);
  return this->addNode(loc, _state, _neighbors);
}

Node* Graph::addNode(const geometry_msgs::Point &loc,
        const Node::State &_state,
        const std::vector<Node*> _neighbors)
{
  // Check if we have a node there already
  for (std::list<Node>::iterator it = this->nodes_.begin(); it != this->nodes_.end(); ++it)
  {
    Node* temp_node = &(*it);
    if (this->CalculateDistance(loc, temp_node->location) < this->lock_percentage_ * this->path_distance_)
    {
      // _neighbors size equal to 1 would mean that this node was created naturaly, not from merging
      if (_neighbors.size() == 1)
      {
        this->MakeNeighbors(_neighbors[0], temp_node);
      }
      return temp_node;
    }
  }

  // Create and fill new node
  Node new_node;
  new_node.name = this->robot_name_;
  std::ostringstream stream_string;
  stream_string << new_node.name << this->nextId_;;
  new_node.id = stream_string.str();
  new_node.location = loc;
  new_node.vizualization_id = this->nextId_;
  if (this->debug_)
  {
    int r = 0, g = 0, b = 0;
    if (_state == Node::State::kUnexplored)
    {
      r = 0.1;
      b = 0.1;
      g = 1;
    }
    else if (_state == Node::State::kExplored)
    {
      r = 1;
      b = 1;
      g = 1;
    }
    this->DrawNode(new_node.location, r, g, b, this->nextId_);
  }
  new_node.state = _state;
  new_node.neighbors = _neighbors;
  this->nodes_.push_back(new_node);

  // Create new map element pointing towards the new node
  Node* new_address = &this->nodes_.back();

  std::pair<std::string,Node*> new_map_element(stream_string.str(), new_address);
  this->nodes_by_id_.insert(new_map_element);

  // Update all neighbors to have a path to this new node
  for (std::vector<Node*>::const_iterator it = _neighbors.begin()  ; it != _neighbors.end(); ++it)
  {
    Node* neighbor = (*it);
    neighbor->neighbors.push_back(new_address);
    neighbor->neighbors_by_id.insert(new_node.id );
    new_address->neighbors_by_id.insert(neighbor->id);
    if (this->debug_)
    {
      this->DrawPath(new_node.location, neighbor->location, 1.0, 1.0, 1.0, this->debug_path_id_++);
    }
  }
  this->nextId_++;

  return new_address;
}

Path Graph::ComputePath(const Node *_begin, const Node *_end)
{
  const std::lock_guard<std::mutex> lock(this->graph_mutex_);
  std::vector<std::string> seen;
  seen.push_back(_begin->id);

  std::unordered_map<std::string, Node*> parents;

  std::unordered_map<std::string, double> g_score;
  g_score.insert({_begin->id, 0});

  std::unordered_map<std::string, double> f_score;
  f_score.insert({_begin->id, this->CalculateDistance(_begin->location , _end->location)});

  while (seen.size() > 0)
  {
    int current_id = this->MinScore(seen, f_score);
    Node* current = this->nodes_by_id_[seen[current_id]];
    if (current->id == _end->id)
    {
      Path complete_path;
      complete_path.push_back(this->nodes_by_id_[_end->id]);
      while (parents.count(current->id) > 0)
      {
        current = parents[current->id];
        complete_path.push_back(current);
      }
      return complete_path;
    }
    seen.erase(seen.begin() + current_id);
    std::vector<Node*> neighbors = current->neighbors;
    for (size_t i = 0; i < neighbors.size(); i++)
    {
      Node* neighbor = neighbors[i];
      if (!neighbor->isAccessible())
      {
        continue;
      }
      if (g_score.count(current->id) <= 0)
      {
        g_score[current->id] = std::numeric_limits<double>::infinity();
      }
      if (g_score.count(neighbor->id) <= 0)
      {
        g_score[neighbor->id] = std::numeric_limits<double>::infinity();
      }
      double tentative_g_score = g_score[current->id] + this->CalculateDistance(current->location, neighbor->location);
      if (tentative_g_score < g_score[neighbor->id])
      {
        parents[neighbor->id] = current;
        g_score[neighbor->id] = tentative_g_score;
        f_score[neighbor->id] = g_score[neighbor->id] + this->CalculateDistance(current->location, _end->location);
        if (std::find(seen.begin(), seen.end(), neighbor->id) == seen.end())
        {
          seen.push_back(neighbor->id);
        }
      }
    }
  }
  return Path();
}

double Graph::CalculateDistance(const geometry_msgs::Point & start, const geometry_msgs::Point & goal) const
{
  return sqrt(pow(start.x - goal.x, 2.0) + pow(start.y - goal.y, 2.0));
}

int Graph::MinScore(const std::vector<std::string> & seen, const std::unordered_map<std::string, double> & f_score) const
{
  double minScore = std::numeric_limits<double>::infinity();
  int minScoreId = -1;
  for (int j = 0; j < seen.size(); j++)
  {
    if (f_score.count(seen[j]) > 0)
    {
      double newScore = f_score.at(seen[j]);
      if (newScore < minScore)
      {
        minScore = newScore;
        minScoreId = j;
      }
    }
  }
  return minScoreId;
}

std::list<Node*> Graph::GetClosestUnexploredNodes(const Node *_begin)
{
  std::list<Node*> close_nodes;
  int minNumNodes = std::numeric_limits<int>::max();
  for (std::list<Node>::iterator it = this->nodes_.begin()  ; it != this->nodes_.end(); ++it)
  {
    Node* testNode = &*it;
    if (testNode->isUnexplored())
    {
      int numNodesInPath = this->ComputePath(_begin, testNode).size();
      if (numNodesInPath <= 0)
      {
        continue;
      }
      if (numNodesInPath == minNumNodes)
      {
        close_nodes.push_back(testNode);
      }
      else if (numNodesInPath < minNumNodes)
      {
        close_nodes = {};
        close_nodes.push_back(testNode);
        minNumNodes = numNodesInPath;
      }
    }
  }
  return close_nodes;
}

float Graph::ComputeOrientation(const Node *_from, const Node *_current, const Node *_destiny)
{
  geometry_msgs::Point location_vector;
  location_vector.x = _current->location.x - _from->location.x;
  location_vector.y = _current->location.y - _from->location.y;
  geometry_msgs::Point destiny_vector;
  destiny_vector.x = _destiny->location.x - _current->location.x;
  destiny_vector.y = _destiny->location.y - _current->location.y;
  double numerator = location_vector.x * destiny_vector.x + location_vector.y * destiny_vector.y;
  double denominator = sqrt(pow(location_vector.x, 2) + pow(location_vector.y, 2)) *
    sqrt(pow(destiny_vector.x, 2) + pow(destiny_vector.y, 2));
  return numerator/denominator;
}

void Graph::MergeGraph(const std::list<Node> & mergin_graph)
{
  ROS_INFO("GraphController::Received graph to merge, starting merging....");
  const std::lock_guard<std::mutex> lock(this->graph_mutex_);
  std::unordered_set<std::string> new_generated_nodes;
  for (std::list<Node>::const_iterator it_g2 = mergin_graph.begin(); it_g2 != mergin_graph.end(); ++it_g2)
  {
    const Node* g2_node = &(*it_g2);
    if (this->nodes_by_id_.find(g2_node->id) != this->nodes_by_id_.end())
    {
      if (g2_node->state == Node::State::kExplored && this->nodes_by_id_[g2_node->id]->state == Node::State::kUnexplored)
      {
        this->UpdateNodeState(this->nodes_by_id_[g2_node->id], Node::State::kExplored);
      }
      continue;
    }
    bool found = false;
    for (std::list<Node>::iterator it_g1 = this->nodes_.begin(); it_g1 != this->nodes_.end(); ++it_g1)
    {
      Node* g1_node = &(*it_g1);
      if (this->CalculateDistance(g1_node->location, g2_node->location) < this->lock_percentage_ * this->path_distance_)
      {
        std::pair<std::string,Node*> new_map_element(g2_node->id, g1_node);
        this->nodes_by_id_.insert(new_map_element);
        found = true;
        break;
      }
    }
    if (!found)
    {
      Node* new_node = this->addNode(g2_node->location, g2_node->state);
      new_node->neighbors_by_id = g2_node->neighbors_by_id;
      std::pair<std::string,Node*> new_map_element(g2_node->id, new_node);
      this->nodes_by_id_.insert(new_map_element);
      new_generated_nodes.insert(g2_node->id);
    }
  }
  for (std::list<Node>::iterator it = this->nodes_.begin(); it != this->nodes_.end(); ++it)
  {
    Node* node = &(*it);
    // Fill the paths of the nodes
    for (std::unordered_set<std::string>::iterator neighbor_it = node->neighbors_by_id.begin(); neighbor_it != node->neighbors_by_id.end(); ++neighbor_it)
    {
      const std::string neighbor_id = *neighbor_it;
      if (this->nodes_by_id_.find(neighbor_id) == this->nodes_by_id_.end())
      {
        continue;
      }
      Node* neighbor = this->nodes_by_id_[neighbor_id];
      this->MakeNeighbors(node, neighbor);

      // Check if this new path is between an added node and a pre-existed node
      if (new_generated_nodes.find(node->id) != new_generated_nodes.end() &&
          new_generated_nodes.find(neighbor_id) == new_generated_nodes.end())
      {
        // TODO(Lobotuerk) This was to take into account diferent type of robots, but does not seem to work, iterate further if needed
        // If pre-existing node was explored, this could mean that merged node cant be reached by this robot, so we mark it inaccessible
        // if (neighbor->state == Node::State::kExplored && node->state == Node::State::kExplored)
        // {
        //   this->UpdateNodeState(node, Node::State::kInaccessible);
        // }
        // If pre-existing node was unexplored, we mark it as explored to avoid exploring it multiple times
        if (neighbor->state == Node::State::kUnexplored)
        {
          this->UpdateNodeState(neighbor, Node::State::kExplored);
        }
      }
    }
  }
  ROS_INFO("GraphController::Finished merging....");
}

void Graph::MakeNeighbors(Node* node1, Node* node2)
{
  if (this->debug_)
  {
    this->DrawPath(node1->location, node2->location, 1.0, 1.0, 1.0, this->debug_path_id_++);
  }
  // Check if the path is known
  if (node2->neighbors_by_id.find(node1->id) == node2->neighbors_by_id.end())
  {
    node2->neighbors_by_id.insert(node1->id);
  }
  if (std::find(node2->neighbors.begin(), node2->neighbors.end(), node1) == node2->neighbors.end())
  {
    node2->neighbors.push_back(node1);
  }
  if (std::find(node1->neighbors.begin(), node1->neighbors.end(), node2) == node1->neighbors.end())
  {
    node1->neighbors.push_back(node2);
  }
}

std::list<Node> Graph::GetGraph() const
{
  return this->nodes_;
}

void Graph::SetGraph(std::list<Node> nodes)
{
  this->nodes_ = nodes;
}

void Graph::PrintGraph() const
{
  ROS_INFO("PRINTING GRAPH");
  for (std::list<Node>::const_iterator iterator = this->nodes_.begin(); iterator != this->nodes_.end(); ++iterator)
  {
    const Node temp = *iterator;
    std::ostringstream stream_string;
    stream_string << "NODE " << temp.id << ":" << static_cast<std::underlying_type<Node::State>::type>(temp.state) << " on " << temp.location.x << ", " << temp.location.y << ", " << temp.location.z << " connects by pointer to [";
    for (std::vector<Node*>::const_iterator neighbors = temp.neighbors.begin(); neighbors != temp.neighbors.end(); ++neighbors)
    {
      stream_string << (*neighbors)->id << ", ";
    }
    stream_string << "]";
    stream_string << " and by id to [";
    for (std::unordered_set<std::string>::const_iterator neighbors = temp.neighbors_by_id.begin(); neighbors != temp.neighbors_by_id.end(); ++neighbors)
    {
      stream_string << (*neighbors) << ", ";
    }
    stream_string << "]";
    ROS_INFO_STREAM(stream_string.str());
  }
}

void Graph::UpdateNodeState(const Node* _cnode, const Node::State &_state)
{
  Node* node = this->nodes_by_id_[_cnode->id];
  node->state = _state;
  if (this->debug_)
  {
    int r = 0, g = 0, b = 0;
    if (_state == Node::State::kUnexplored)
    {
      r = 0.1;
      b = 0.1;
      g = 1;
    }
    else if (_state == Node::State::kExplored)
    {
      r = 1;
      b = 1;
      g = 1;
    }
    this->DrawNode(node->location, r, g, b, node->vizualization_id);
  }
}

void Graph::DrawNode(const geometry_msgs::Point & point, float r, float g, float b, int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = this->artifact_origin_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "graph_nodes";
  marker.id = id;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.color.a = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.points.push_back(point);
  this->debug_pub_.publish(marker);
}

void Graph::DrawPath(const geometry_msgs::Point & point1, const geometry_msgs::Point & point2, float r, float g, float b, int id)
{
  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = this->artifact_origin_frame_;
  path_marker.header.stamp = ros::Time::now();
  path_marker.ns = "graph_paths";
  path_marker.id = id;
  path_marker.type = visualization_msgs::Marker::LINE_LIST;
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.pose.orientation.x = 0.0;
  path_marker.pose.orientation.y = 0.0;
  path_marker.pose.orientation.z = 0.0;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = 0.05;
  path_marker.color.a = 1.0;
  path_marker.color.r = r;
  path_marker.color.g = g;
  path_marker.color.b = b;
  path_marker.points.push_back(point1);
  path_marker.points.push_back(point2);
  this->debug_pub_.publish(path_marker);
}

void Graph::SetRobotName(const std::string & robot_name)
{
  this->robot_name_ = robot_name;
}

}  // namespace graph_controller
