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

#ifndef ARTIFACT_LOCALIZATION_ARTIFACT_DATABASE_H_
#define ARTIFACT_LOCALIZATION_ARTIFACT_DATABASE_H_

#include "Artifact.hh"
#include "ros/ros.h"

/// \class ArtifactDatabase ArtifactDatabase.hh
/// \brief Class for storing and modifying all artifacts that have been found.
class ArtifactDatabase
{
  /// \brief Contstructor
  public: ArtifactDatabase();

  /// \brief Inserts a detection into the database.
  /// If the detection's location does not match an existing artifact's location,
  /// a new artifact is created from the detection.
  /// If the detection's location does match an existing artifact's location,
  /// the detection is added to this artifact.
  /// \param[in] detection The detection to be added to the database.
  public: void InsertDetection(const Detection & detection);

  /// \brief Gets all artifacts that are in the database.
  /// \returns The artifacts in the database.
  public: std::vector<Artifact> GetArtifacts() const;

  /// \brief Marks an artifact in the database as reported.
  /// \param[in] artifactIdx The vector index of the artifact to be marked as reported.
  public: void MarkArtifactAsReported(const size_t artifactIdx);

  /// \brief Displays an artifact as a marker in rviz.
  /// \param[in] artifactIdx The vector index of the artifact to be displayed.
  /// \param[in] isNewArtifact An indication of whether the artifact is being displayed for the first time or not.
  /// If an artifact is not being displayed for the first time, the location of the current marker must be modified.
  private: void DisplayArtifactMarker(const size_t artifactIdx, const bool isNewArtifact) const;

  /// \brief The database of artifacts, stored in a list.
  private: std::vector<Artifact> artifacts;

  /// \brief ROS nodehandle. Needed for publishing the artifacts as markers in rviz.
  private: ros::NodeHandle nh;

  /// \brief ROS publisher. Used to publish markers in rviz.
  private: ros::Publisher pub;

  /// \brief The name of the artifact origin frame.
  private: std::string artifact_origin_frame_;
};

#endif
