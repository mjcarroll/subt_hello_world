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

#include "artifact_localization/ArtifactDatabase.hh"
#include "subt_ign/CommonTypes.hh"
#include "visualization_msgs/Marker.h"

ArtifactDatabase::ArtifactDatabase()
{
  std::string nodeName = ros::this_node::getName();
  this->nh.param<std::string>(nodeName + "/artifact_origin_frame", this->artifact_origin_frame_, "artifact_origin");

  this->pub = nh.advertise<visualization_msgs::Marker>("artifact_marker", 0);
}

void ArtifactDatabase::InsertDetection(const Detection & detection)
{
  // check to see if this detection is of an artifact that has already been seen
  for (size_t i = 0; i < this->artifacts.size(); ++i)
  {
    auto artifact = this->artifacts[i];
    if ((detection.GetType() == artifact.GetType()) && artifact.IsSameLocation(detection.GetLocation()))
    {
      artifact.AddDetection(detection);
      this->DisplayArtifactMarker(i, false);
      return;
    }
  }

  // make a new artifact if this detection doesn't match up with already-seen artifacts
  this->artifacts.push_back(Artifact(detection));
  this->DisplayArtifactMarker(this->artifacts.size() - 1, true);
}

std::vector<Artifact> ArtifactDatabase::GetArtifacts() const
{
  return this->artifacts;
}

void ArtifactDatabase::MarkArtifactAsReported(const size_t artifactIdx)
{
  this->artifacts[artifactIdx].MarkAsReported();
}

void ArtifactDatabase::DisplayArtifactMarker(const size_t artifactIdx, const bool isNewArtifact) const
{
  auto artifact = this->artifacts[artifactIdx];
  auto location = artifact.Location();
  auto isBackpack = artifact.GetType() == subt::ArtifactType::TYPE_BACKPACK;
  auto isPerson = artifact.GetType() == subt::ArtifactType::TYPE_RESCUE_RANDY;
  auto isRope = artifact.GetType() == subt::ArtifactType::TYPE_ROPE;

  visualization_msgs::Marker marker;
  marker.header.frame_id = artifact_origin_frame_;
  marker.header.stamp = ros::Time();
  marker.ns = isBackpack ? "backpacks" : isPerson ? "people" : isRope? "ropes" : "detected_artifacts";
  marker.id = artifactIdx;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = isNewArtifact ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::MODIFY;
  marker.pose.position.x = location.x;
  marker.pose.position.y = location.y;
  marker.pose.position.z = location.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = isBackpack || isPerson ? 1.0 : 0.0;
  marker.color.g = isPerson ? 1.0 : 0.0;
  marker.color.b = isRope ? 1.0 : 0.0;

  this->pub.publish(marker);
}
