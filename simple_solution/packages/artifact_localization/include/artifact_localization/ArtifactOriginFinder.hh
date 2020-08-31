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

#ifndef ARTIFACT_LOCALIZATION_ARTIFACT_ORIGIN_FINDER_H_
#define ARTIFACT_LOCALIZATION_ARTIFACT_ORIGIN_FINDER_H_

#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "subt_msgs/PoseFromArtifact.h"
#include <string>
#include "geometry_msgs/TransformStamped.h"

/// \class ArtifactOriginFinder ArtifactOriginFinder.hh
/// \brief Class for finding the artifact origin and saving the static transform
/// between the map frame and artifact origin frame.
class ArtifactOriginFinder
{
  /// \brief Constructor
  public: ArtifactOriginFinder();

  /// \brief Finds the artifact origin and broadcasts a static transform between
  /// the map frame and artifact origin frame
  private: void FindArtifactOrigin();

  /// \brief Broadcasts the transform between the map and artifact origin at a pre-defined rate.
  private: void BroadcastArtifactOriginTF();

  /// \brief The name of the service that gets the location of the artifact origin.
  private: const std::string serviceName;

  /// \brief ROS nodehandle. Needed to call the artifact origin service.
  private: ros::NodeHandle nh;

  /// \brief Static transform broadcaster that saves the static transform between the
  /// map frame and artifact origin frame.
  private: tf2_ros::StaticTransformBroadcaster staticTfBroadcaster;

  /// \brief Buffer for transform listener that ensures the map frame is available before calling
  /// the artifact origin service. Also used to get the transform between the map frame and artifact origin
  /// so that the static transform broadcaster can save it.
  private: tf2_ros::Buffer tfBuffer;

  /// \brief Transform listener. Initializes tfBuffer.
  private: tf2_ros::TransformListener tfListener;

  /// \brief The transformation between the map and artifact origin.
  private: geometry_msgs::TransformStamped transformStamped;

  /// \brief Used to broadcast the artifact origin location at a pre-defined rate.
  private: ros::Timer timer;

  /// \brief The name of the map frame.
  private: std::string map_frame_;

  /// \brief The name of the artifact origin frame.
  private: std::string artifact_origin_frame_;

  /// \brief The name of the base link frame.
  private: std::string base_link_frame_;

  /// \brief The name of the robot.
  /// This is needed for the artifact origin service call.
  private: std::string robot_name_;
};

#endif
