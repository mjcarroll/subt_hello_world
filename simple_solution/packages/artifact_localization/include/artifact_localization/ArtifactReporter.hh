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

#ifndef ARTIFACT_LOCALIZATION_ARTIFACT_REPORTER_H_
#define ARTIFACT_LOCALIZATION_ARTIFACT_REPORTER_H_

#include "ArtifactDatabase.hh"
#include <artifact_localization_msgs/ReportArtifacts.h>
#include <std_msgs/String.h>

/// \class ArtifactReporter ArtifactReporter.hh
/// \brief Class for reporting artifacts to the base station.
class ArtifactReporter
{
  /// \brief Constructor
  public: ArtifactReporter();

  /// \brief Adds a detection to the artifact database.
  /// \param[in] detection The detection to be added to the artifact database.
  public: void AddDetectionToDB(const Detection & detection);

  /// \brief Reports artifacts marked as unreported to the base station.
  private: void ReportArtifacts();

  /// \brief Attempt to send an unreported artifact to the base station.
  /// \param[in] artifact The unreported artifact that still needs to be reported to the base station.
  private: void ReportArtifact(const Artifact & artifact);

  /// \brief A callback function that serves as a notification of when an artifact report
  /// has been received by the base station.
  /// \param[in] msg The message from the base station.
  void BaseStationCB(const std_msgs::String::ConstPtr & msg);

  /// \brief Publishes the number of unreported artifacts to a ROS topic.
  private: void BroadcastAvailableArtifacts();

  /// \brief Responds to a report service request by calling the ReportArtifacts() method.
  /// \param req The service request
  /// \param resp The service response
  /// \return true once the reporting attempt was made
  private: bool ReportServiceCB(
    artifact_localization_msgs::ReportArtifacts::Request & req,
    artifact_localization_msgs::ReportArtifacts::Response & resp
  );

  /// \brief ROS nodehandle. Needed to set up a timer callback.
  private: ros::NodeHandle nh;

  /// \brief The name of the robot.
  /// This is needed for the artifact origin service call.
  private: std::string robot_name_;

  /// \brief A record of all artifacts that have been found, including their reported state.
  private: ArtifactDatabase artifactDatabase;

  /// \brief ROS publisher. Broadcasts the number of unreported artifacts.
  /// Publishes data to the "output/unreported_artifacts" topic.
  private: ros::Publisher unreported_artifacts_pub;

  /// \brief Publishes the total number of detected artifacts (includes
  /// correct and incorrect detections).
  private: ros::Publisher total_artifacts_pub;

  /// \brief Service server that listens to requests for reporting artifacts
  private: ros::ServiceServer service;

  /// \brief Publishes a serialized artifact report to a ROS topic
  private: ros::Publisher artifact_report_pub;

  /// \brief Subscribes to a topic containing the base station's artifact report response
  private: ros::Subscriber base_station_sub;
};

#endif
