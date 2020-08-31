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

#include <subt_solution_comms/comms_manager.hh>

#include <functional>
#include <memory>

#include <subt_ign/CommonTypes.hh>

CommsManager::CommsManager()
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  ros::NodeHandle private_nh("~");

  int graph_port, pose_port;
  private_nh.param<int>("graph_port", graph_port, 5000);
  private_nh.param<int>("pose_port", pose_port, 5001);

  // initialize the CommsClient
  std::string robot_name;
  private_nh.param<std::string>("name", robot_name, "X1");
  this->comms_client_ = std::make_unique<subt::CommsClient>(robot_name);

  // initialize all local communicators
  auto outer_comms_func = std::bind(&CommsManager::Communicate, this, _1);
  this->artifact_comms_ = std::make_unique<LocalCommunicator>(outer_comms_func,
                                                              "artifact_report_result",
                                                              "detected_artifacts",
                                                              subt::kBaseStationName,
                                                              subt::kDefaultPort,
                                                              robot_name);
  this->graph_comms_ = std::make_unique<LocalCommunicator>(outer_comms_func,
                                                           "other_robot_graph",
                                                           "local_robot_graph",
                                                           subt::kBroadcast,
                                                           graph_port,
                                                           robot_name);
  this->pose_comms_ = std::make_unique<LocalCommunicator>(outer_comms_func,
                                                          "other_robot_pose",
                                                          "local_robot_pose",
                                                          subt::kBroadcast,
                                                          pose_port,
                                                          robot_name);

  // Bind message callbacks to a port that's related to the message types being sent on that port
  this->comms_client_->Bind(std::bind(&LocalCommunicator::IncomingMsgCB, this->artifact_comms_.get(), _1, _2, _3, _4),
                            robot_name,
                            this->artifact_comms_->Port());
  this->comms_client_->Bind(std::bind(&LocalCommunicator::IncomingMsgCB, this->graph_comms_.get(), _1, _2, _3, _4),
                            robot_name,
                            this->graph_comms_->Port());
  this->comms_client_->Bind(std::bind(&LocalCommunicator::IncomingMsgCB, this->pose_comms_.get(), _1, _2, _3, _4),
                            robot_name,
                            this->pose_comms_->Port());
}

void CommsManager::Communicate(const CommsInfo & comms_info)
{
  ROS_DEBUG_STREAM("CommsManager::Communicate: got data from a LocalCommunicator, forwarding to an external component");
  this->comms_client_->SendTo(comms_info.data, comms_info.dst_address, comms_info.port);
}
