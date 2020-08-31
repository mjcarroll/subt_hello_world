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

#ifndef COMMS_COMMS_MANAGER_H
#define COMMS_COMMS_MANAGER_H

#include <memory>
#include <string>

#include <subt_communication_broker/subt_communication_client.h>

#include <subt_solution_comms/local_communicator.hh>

/// \class CommsManager comms_manager.hh
/// \brief Class that manages communication between internal components of a robot,
/// and external components (base station and other robots)
class CommsManager
{
  /// \brief Constructor
  public: CommsManager();

  /// \brief Communicate with an external component
  /// \param[in] comms_info The information that describes what component
  /// to communicate with, along with the data to be shared
  public: void Communicate(const CommsInfo & comms_info);

  /// \brief SubT comms client
  private: std::unique_ptr<subt::CommsClient> comms_client_;

  /// \brief Handles communications with the local components that deal with artifacts
  private: std::unique_ptr<LocalCommunicator> artifact_comms_;

  /// \brief Handles communications regarding this robot's and other robot's graphs
  private: std::unique_ptr<LocalCommunicator> graph_comms_;

  /// \brief Handles communications regarding this robot's and other robot's poses
  private: std::unique_ptr<LocalCommunicator> pose_comms_;
};

#endif
