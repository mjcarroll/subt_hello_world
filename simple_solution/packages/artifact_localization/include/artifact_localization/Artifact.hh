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

#ifndef ARTIFACT_LOCALIZATION_ARTIFACT_H_
#define ARTIFACT_LOCALIZATION_ARTIFACT_H_

#include "Detection.hh"
#include <vector>

/// \class Artifact Artifact.hh
/// \brief Class for storing an artifact, formed by one or more detections.
class Artifact
{
  /// \brief Constructor
  /// \param[in] detection A detection that determines the artifact's location and type.
  /// \param[in] locationTolerance The distance, in meters, that determines if 2 detections belong to the same artifact.
  public: Artifact(const Detection & detection, double locationTolerance = 8.0);

  /// \brief Adds a detection to an artifact, which will have an effect on the artifact's position.
  /// \param[in] detection The detection to be added to an artifact.
  public: void AddDetection(const Detection & detection);

  /// \brief Gets the location of the artifact, which is the average location of all the artifact's detections.
  /// \returns The location of the artifact.
  public: geometry_msgs::Point Location() const;

  /// \brief Determines if a location is in the same location as an already identified artifact.
  /// \param[in] location The location to be tested in comparison to the artifact's location.
  /// \returns True if location is within locationTolerance of the artifact's location. False otherwise.
  public: bool IsSameLocation(const geometry_msgs::Point & location) const;

  /// \brief Gets the type of the artifact.
  /// \returns The artifact type.
  public: subt::ArtifactType GetType() const;

  /// \brief Determines if the artifact has been successfully reported to the base station.
  /// \returns True if the artifact has been successfully reported. False otherwise.
  public: bool IsReported() const;

  /// \brief Sets the isReported data member of the artifact object instance to true.
  public: void MarkAsReported();

  /// \brief The detection objects that define the artifact.
  private: std::vector<Detection> detections;

  /// \brief The status of whether the artifact has been successfully reported to the base station or not.
  private: bool isReported;

  /// \brief The maximum distance, in meters, that determines if other locations are related to the artifact or not.
  private: double locationTolerance;

  /// \brief The type of the artifact.
  private: subt::ArtifactType type;
};

#endif
