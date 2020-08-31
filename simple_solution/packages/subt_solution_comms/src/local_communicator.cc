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

#include <subt_solution_comms/local_communicator.hh>

#include <ignition/common/Util.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>

LocalCommunicator::LocalCommunicator(OuterCommsFunc outer_comms_func,
                                     const std::string & publish_topic,
                                     const std::string & subscribe_topic,
                                     const std::string & outside_dst_address,
                                     const int port,
                                     const std::string & robot_name) :
  outer_comms_func_(outer_comms_func),
  publisher_(this->nh_.advertise<std_msgs::String>(publish_topic, 10)),
  subscriber_(this->nh_.subscribe(subscribe_topic, 10, &LocalCommunicator::SubscriberCB, this)),
  outside_dst_address_(outside_dst_address),
  port_(port),
  robot_name_(robot_name)
{}

int LocalCommunicator::Port() const
{
  return this->port_;
}

void LocalCommunicator::IncomingMsgCB(const std::string & src_addr,
                                      const std::string & dst_addr,
                                      const uint32_t dst_port,
                                      const std::string & data)
{
  // ignore messages from yourself
  // (this can happen with messages sent on subt::kBroadcast)
  if (src_addr == this->robot_name_)
  {
    return;
  }

  // ignore a repeated message
  // (hack that fixes the duplicate callback trigger issue for subt::kBroadcast)
  auto hash = ignition::common::hash64(data);
  if (this->msg_hashes.find(hash) != this->msg_hashes.end())
  {
    return;
  }

  this->msg_hashes.insert(hash);

  ROS_DEBUG_STREAM("LocalCommunicator::IncomingMsgCB: Received a new message from "
                  << src_addr << " with a dst_port of " << dst_port
                  << ". Publishing to this message to a topic intended for "
                  << dst_addr);

  std_msgs::String msg;
  msg.data = data;
  this->publisher_.publish(msg);
}

void LocalCommunicator::SubscriberCB(const std_msgs::String::ConstPtr & msg)
{
  ROS_DEBUG("LocalCommunicator::SubscriberCB: Received a message, forwarding to CommsManager");

  CommsInfo comms_info;
  comms_info.data = msg->data;
  comms_info.dst_address = this->outside_dst_address_;
  comms_info.port = this->port_;

  this->outer_comms_func_(comms_info);
}
