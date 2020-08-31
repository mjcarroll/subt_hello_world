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

#ifndef COMMS_LOCAL_COMMUNICATOR_H
#define COMMS_LOCAL_COMMUNICATOR_H

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_set>

#include <ros/ros.h>
#include <std_msgs/String.h>

/// \brief Information that is needed for communication with external components
struct CommsInfo
{
  std::string data;
  std::string dst_address;
  int port;
};

/// \class LocalCommunicator local_communicator.hh
/// \brief Class that manages communication with an internal component of a robot
class LocalCommunicator
{
  /// \brief Function that is passed into the LocalCommunicator object that helps
  /// relay local information to external components
  using OuterCommsFunc = std::function<void(const CommsInfo & comms_info)>;

  /// \brief Constructor
  /// \param[in] outer_comms_func The function to be called that relays information
  /// to external components
  /// \param[in] publish_topic The local topic this object should publish data to.
  /// Used to tell local components about new data
  /// \param[in] subscribe_topic The local topic this object should subscribe to.
  /// Used to get new information from a local component
  /// \param[in] outside_dst_address The address of an external component that this
  /// object can relay information to
  /// \param[in] port The port to be used with this->outside_dst_address_
  /// \param[in] robot_name The name of the robot that's being used for local communications
  public: LocalCommunicator(OuterCommsFunc outer_comms_func,
                            const std::string & publish_topic,
                            const std::string & subscribe_topic,
                            const std::string & outside_dst_address,
                            const int port,
                            const std::string & robot_name);

  /// \brief Get the port for this object's corresponding external component
  public: int Port() const;

  /// \brief The callback an incoming message from the SubT communications framework
  /// \param[in] src_addr The address of the sender
  /// \param[in] dst_addr The address of the destination
  /// \param[in] dst_port The destination port that should be used
  /// \param[in] data The data that was sent, serialized as a string
  public: void IncomingMsgCB(const std::string & src_addr,
                             const std::string & dst_addr,
                             const uint32_t dst_port,
                             const std::string & data);

  /// \brief Callback that is triggered whenever new data is published to the subscribed local topic
  /// \param[in] msg The data that was published to the local topic
  private: void SubscriberCB(const std_msgs::String::ConstPtr & msg);

  /// \brief Function that allows the LocalCommunicator to forward information to a higher-level
  /// communications manager
  private: OuterCommsFunc outer_comms_func_;

  /// \brief ROS objects that are used to publish/subscribe to local component topics
  private: ros::NodeHandle nh_;
  private: ros::Publisher publisher_;
  private: ros::Subscriber subscriber_;

  /// \brief Information that defines what external component the LocalCommunicator can relay info to
  private: std::string outside_dst_address_;
  private: int port_;

  /// \brief The name of the robot that is being used for local communications
  const std::string robot_name_;

  /// \brief Hash codes for all received messages (handles duplicates)
  private: std::unordered_set<std::uint64_t> msg_hashes;
};

#endif
