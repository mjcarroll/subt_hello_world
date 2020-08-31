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

#include "artifact_localization/Artifact.hh"
#include <math.h>

Artifact::Artifact(const Detection & detection, double locationTolerance) :
  detections{detection},
  isReported(false),
  locationTolerance(locationTolerance),
  type(detection.GetType())
{}

void Artifact::AddDetection(const Detection & detection)
{
  this->detections.push_back(detection);
}

geometry_msgs::Point Artifact::Location() const
{
  geometry_msgs::Point avg;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  for (const auto & detection : this->detections)
  {
    x += detection.GetLocation().x;
    y += detection.GetLocation().y;
    z += detection.GetLocation().z;
  }
  x /= detections.size();
  y /= detections.size();
  z /= detections.size();

  avg.x = x;
  avg.y = y;
  avg.z = z;

  return avg;
}

bool Artifact::IsSameLocation(const geometry_msgs::Point & location) const
{
  auto currLocation = this->Location();

  double xDiff = currLocation.x - location.x;
  double yDiff = currLocation.y - location.y;
  double zDiff = currLocation.z - location.z;
  auto dist = sqrt((xDiff * xDiff) + (yDiff * yDiff) + (zDiff * zDiff));

  return dist <= this->locationTolerance;
}

subt::ArtifactType Artifact::GetType() const
{
  return this->type;
}

bool Artifact::IsReported() const
{
  return this->isReported;
}

void Artifact::MarkAsReported()
{
  this->isReported = true;
}
