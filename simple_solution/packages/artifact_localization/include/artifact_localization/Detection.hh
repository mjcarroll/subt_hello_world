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

#ifndef ARTIFACT_LOCALIZATION_DETECTION_H_
#define ARTIFACT_LOCALIZATION_DETECTION_H_

#include "geometry_msgs/Point.h"
#include <string>
#include "subt_ign/CommonTypes.hh"

/// \class Detection Detection.hh
/// \brief Class for storing a detection from darknet.
class Detection
{
  /// \brief Constructor
  /// \param[in] location Location of the detection
  /// \param[in] confidence Confidence of the detection type
  /// \param[in] type Type of object that was detected
  public: Detection(
    const geometry_msgs::Point & location,
    const double confidence,
    const subt::ArtifactType type);

  /// \brief Gets the location of the detection
  /// \returns The location of the detection.
  public: geometry_msgs::Point GetLocation() const;

  /// \brief Gets the confidence of the detection type
  /// \returns The confidence of the detection.
  public: double GetConfidence() const;

  /// \brief Gets the type of the detection
  /// \returns The type of the detection.
  public: subt::ArtifactType GetType() const;

  /// \brief The location of the detection
  private: geometry_msgs::Point location;

  /// \brief The confidence in the type of object that was detected
  private: double confidence;

  /// \brief The type of object that was detected
  private: subt::ArtifactType type;
};

#endif
