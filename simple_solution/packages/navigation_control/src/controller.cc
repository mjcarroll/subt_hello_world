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

#include <graph_controller/controller.hh>

#include <math.h>

#include <geometry_msgs/Point.h>
#include <ignition/msgs/pose.pb.h>

namespace graph_controller {

Controller::Controller(const std::string & robot_name,
                       const std::string & artifact_origin_frame,
                       std::function<void()> extern_bt_func) :
  graph_(),
  motion_controller_(std::bind(&Controller::MotionCallback, this, std::placeholders::_1),
                               this->nh_.param<int>(ros::this_node::getName() + "/node_timeout", 60)),
  robotName_(robot_name),
  artifact_origin_frame_(artifact_origin_frame),
  frontiers_finder_(this->nh_.param<float>(ros::this_node::getName() + "/path_distance", 5.0), 0.5, 4.0, 0.5),
  graph_publisher_(this->nh_.advertise<std_msgs::String>("local_robot_graph", 10)),
  graph_subscriber_(this->nh_.subscribe("other_robot_graph", 10, &Controller::OtherGraphCB, this)),
  back_track_state_(false),
  extern_bt_func_(extern_bt_func)
{
  this->nh_.param<float>(ros::this_node::getName() + "/wait_seconds_before_exploring", this->wait_seconds_before_exploring_, 0.0);
  this->nh_.param<float>(ros::this_node::getName() + "/distance_until_merge", this->distance_until_merge_, 30.0);
  this->nh_.param<float>(ros::this_node::getName() + "/starting_node_x_pos", this->starting_node_x_pos_, 5.0);
  this->nh_.param<float>(ros::this_node::getName() + "/starting_node_y_pos", this->starting_node_y_pos_, 0.0);

  // the artifact origin circle is centered around (0,0)
  this->artifact_origin_space_ = sqrt(pow(this->starting_node_x_pos_, 2.0) + pow(this->starting_node_y_pos_, 2.0));

  // Initialize Graph structure and exploration routine
  this->Initialize();

  // Wait before exploring to allow other robots to enter the cave
  ros::Duration wait_duration(this->wait_seconds_before_exploring_);
  wait_duration.sleep();

  // Start exploration
  this->AdvanceDownPath();
  ROS_DEBUG("Controller up and running");

  this->graph_timer_ = this->nh_.createTimer(ros::Duration(5.0), boost::bind(&Controller::ShareGraph, this));
}

void Controller::Initialize()
{
  // First place a graph node at the artifact origin
  geometry_msgs::Point origin;
  origin.x = 0;
  origin.y = 0;
  origin.z = 0;
  this->artifact_origin_node_ = this->graph_.AddNode(origin, Node::State::kExplored);

  // Initialize our last visited
  this->lastVisited_ = this->artifact_origin_node_;
  this->currentStep_ = this->artifact_origin_node_;

  // Then place a graph node at the mouth of the cave
  geometry_msgs::Point cave_point;
  cave_point.x = this->starting_node_x_pos_;
  cave_point.y = this->starting_node_y_pos_;
  cave_point.z = 0;
  auto cave_entrance = this->graph_.AddNode(cave_point, Node::State::kUnexplored, std::vector<Node*>({this->artifact_origin_node_}));

  this->SetGoal(cave_entrance);
}

void Controller::SetGoal(const Node *_destination)
{
  // Compute a path from our current node to the destination
  this->path_ = this->graph_.ComputePath(this->currentStep_, _destination);
}

void Controller::ComputeFrontiers(Node* _node)
{
  auto frontiers = this->frontiers_finder_.findFrontiers(this->artifact_origin_frame_);
  for (const auto & frontier: frontiers)
  {
    geometry_msgs::Point origin;
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;
    if (this->graph_.CalculateDistance(frontier, origin) < this->artifact_origin_space_)
    {
      continue;
    }
    this->graph_.AddNode(frontier, Node::State::kUnexplored, std::vector<Node*>({_node}));
  }
}

void Controller::MotionCallback(bool successful)
{
  if (successful)
  {
    // Update our last visited node
    if (this->currentStep_->isUnexplored())
    {
      this->graph_.UpdateNodeState(this->currentStep_, Node::State::kExplored);
      this->ComputeFrontiers(this->currentStep_);
    }
    Node* temp_current = this->currentStep_;
    this->AdvanceDownPath();
    // Update last visited and clear current
    this->lastVisited_ = temp_current;
  }
  else
  {
    if (this->currentStep_->isUnexplored())
    {
      this->graph_.UpdateNodeState(this->currentStep_, Node::State::kInaccessible);
    }
    this->AdvanceDownPath();
  }

  this->ShareGraph();
}

Node* Controller::ComputeGoal(Node* _from, Node* _current)
{
  Node* best_node;
  float maxOrientation = -2.0;
  // Get closest nodes to current node
  std::list<Node*> closest_nodes = this->graph_.GetClosestUnexploredNodes(_current);
  if (closest_nodes.size() == 0)
  {
    return nullptr;
  }

  // Choose the node that best follows the orientation of the robot from the closest ones
  for (std::list<Node*>::iterator it = closest_nodes.begin(); it != closest_nodes.end(); ++it)
  {
    Node* testNode = *it;
    float orientation = this->graph_.ComputeOrientation(_from, _current, testNode);
    if (orientation > maxOrientation)
    {
      best_node = testNode;
      maxOrientation = orientation;
    }
  }

  return best_node;
}

void Controller::AdvanceDownPath()
{
  // If the path is used up, it is time to recompute
  if (this->path_.size() == 0)
  {
    this->RecomputePath();
  }

  // Grab the next step from the front of the path
  this->currentStep_ = this->path_.back();
  this->motion_controller_.MoveToPoint(this->currentStep_->location, this->artifact_origin_frame_);
  this->path_.pop_back();
}

void Controller::RecomputePath()
{
  if (this->back_track_state_)
  {
    this->SetGoal(this->artifact_origin_node_);
    return;
  }
  auto goal = this->ComputeGoal(this->lastVisited_, this->currentStep_);
  // If goal is empty, there is no more unexplored places on the graph
  if (nullptr == goal)
  {
    ROS_INFO("No more places to explore, exploration finished");
    this->back_track_state_ = true;
    this->SetGoal(this->artifact_origin_node_);
    // let an outside user know backtracking started early
    // (everything has been explored)
    if (this->extern_bt_func_)
    {
      this->extern_bt_func_();
    }
    return;
  }
  this->SetGoal(goal);
}

void Controller::Backtrack()
{
  this->back_track_state_ = true;
  this->RecomputePath();
}

bool Controller::DoneBacktracking() const
{
  auto dist = this->graph_.CalculateDistance(this->artifact_origin_node_->location,
      this->lastVisited_->location);
  return this->back_track_state_ && (dist <= 10);
}

void Controller::ShareGraph()
{
  ROS_DEBUG_STREAM(this->robotName_ <<
                  " Controller::ShareGraph: sharing graph now...");


 //Split the graph into chunks of 5o nodes each with a maximum size of 1500
  int chunk_size = 50;

  //Max serialized size is the same as the Maximum transmission payload size defined in:
  // https://github.com/osrf/subt/blob/5f7a665ed0936c3a6b65142fa5bc626d316ab886/
  // subt-communication/subt_communication_broker/include/subt_communication_broker/
  // subt_communication_client.h#L202
  int max_serialized_size = 1500;

  size_t serialized_size;
  std::vector<std::string> serialized_chunks;
  std::list<graph_controller::Node> nodes = this->graph_.GetGraph();
  std::list<graph_controller::Node>::iterator chunk_begin = nodes.begin(), chunk_end = nodes.begin();
  std::list<graph_controller::Node> currList;
  std::string serialized;

  while(chunk_end != nodes.end() )
  {

    int serialized_size = max_serialized_size + 1;

    for(int this_chunk_size = chunk_size; serialized_size > max_serialized_size; --this_chunk_size)
    {
      if(this_chunk_size != chunk_size)
      {
        ROS_DEBUG("Resizing chunk");
      }

      chunk_end = chunk_begin;

      for(int i=0; i < this_chunk_size; ++i)
      {
        std::advance(chunk_end, 1);
        if(chunk_end == nodes.end())
        {
          break;
        }
      }

      currList = std::list<graph_controller::Node>( chunk_begin, chunk_end );
      serialized = this->serializer_.SerializeGraph(currList);
      serialized_size = serialized.length();
    }

    chunk_begin = chunk_end;
    serialized_chunks.push_back( serialized );
  }

  //publish chunks

  std_msgs::String msg;
  for(const std::string & str_msg : serialized_chunks)
  {
    msg.data = str_msg;
    this->graph_publisher_.publish(msg);
  }

}

void Controller::OtherGraphCB(const std_msgs::String::ConstPtr & msg)
{
  geometry_msgs::Point origin;
  origin.x = 0;
  origin.y = 0;
  origin.z = 0;
  if (this->graph_.CalculateDistance(this->lastVisited_->location, origin) < this->distance_until_merge_)
  {
    ROS_DEBUG_STREAM(this->robotName_ <<
                    " Controller::OtherGraphCB: received a graph from another robot! But we are too close to origin, skipping");
    return;
  }
  ROS_DEBUG_STREAM(this->robotName_ <<
                  " Controller::OtherGraphCB: received a graph from another robot! Merging graphs...");
  this->graph_.MergeGraph(this->serializer_.DeSerializeGraph(msg->data));
}
}  // namespace graph_controller
