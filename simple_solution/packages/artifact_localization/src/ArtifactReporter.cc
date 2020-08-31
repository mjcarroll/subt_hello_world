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

#include "artifact_localization/ArtifactReporter.hh"
#include "geometry_msgs/Point.h"
#include "ignition/msgs.hh"
#include "subt_ign/CommonTypes.hh"
#include "subt_ign/protobuf/artifact.pb.h"
#include "std_msgs/UInt32.h"

ArtifactReporter::ArtifactReporter() :
  robot_name_(this->nh.param<std::string>(ros::this_node::getName() + "/name", "X1")),
  unreported_artifacts_pub(this->nh.advertise<std_msgs::UInt32>("output/unreported_artifacts", 1)),
  total_artifacts_pub(this->nh.advertise<std_msgs::UInt32>("output/num_detected_artifacts", 1)),
  service(this->nh.advertiseService("report_artifacts", &ArtifactReporter::ReportServiceCB, this)),
  artifact_report_pub(this->nh.advertise<std_msgs::String>("detected_artifacts", 10)),
  base_station_sub(this->nh.subscribe("artifact_report_result", 10, &ArtifactReporter::BaseStationCB, this))
{
}

void ArtifactReporter::AddDetectionToDB(const Detection & detection)
{
  this->artifactDatabase.InsertDetection(detection);
}

void ArtifactReporter::ReportArtifacts()
{
  auto artifacts = this->artifactDatabase.GetArtifacts();
  auto num_artifacts = artifacts.size();
  for (auto i = 0; i < num_artifacts; ++i)
  {
    if (!artifacts[i].IsReported())
    {
      this->ReportArtifact(artifacts[i]);
    }
  }

  // now that an attempt has been made to report all artifacts,
  // publish how many artifacts remain unreported and
  // the number of artifacts found so far
  this->BroadcastAvailableArtifacts();
  std_msgs::UInt32 msg;
  msg.data = num_artifacts;
  this->total_artifacts_pub.publish(msg);
}

void ArtifactReporter::ReportArtifact(const Artifact & artifactToReport)
{
  auto location = artifactToReport.Location();
  ignition::msgs::Pose pose;
  pose.mutable_position()->set_x(location.x);
  pose.mutable_position()->set_y(location.y);
  pose.mutable_position()->set_z(location.z);

  // fill the type and pose
  subt::msgs::Artifact artifact;
  artifact.set_type(static_cast<uint32_t>(artifactToReport.GetType()));
  artifact.mutable_pose()->CopyFrom(pose);

  // serialize the artifact
  std::string serializedData;
  if (!artifact.SerializeToString(&serializedData))
  {
    ROS_ERROR_STREAM("ArtifactReporter::ReportArtifact(): Error serializing message\n" << artifact.DebugString());
  }

  // report the artifact
  std_msgs::String msg;
  msg.data = serializedData;
  this->artifact_report_pub.publish(msg);
}

void ArtifactReporter::BaseStationCB(const std_msgs::String::ConstPtr & msg)
{
  ROS_INFO_STREAM("ArtifactReporter::BaseStationCB: Got a reply from the base station! Marking something as reported...");
  std::string data = msg->data;
  subt::msgs::ArtifactScore res;
  if (!res.ParseFromString(data))
  {
    ROS_ERROR("ArtifactReporter::BaseStationCallback(): error deserializing message.");
  }

  geometry_msgs::Point location;
  location.x = res.artifact().pose().position().x();
  location.y = res.artifact().pose().position().y();
  location.z = res.artifact().pose().position().z();

  // modify which artifact has been reported in the database to make sure that artifacts are only reported once
  auto artifacts = this->artifactDatabase.GetArtifacts();
  for (auto i = 0; i < artifacts.size(); ++i)
  {
    if (artifacts[i].IsReported())
    {
      continue;
    }

    // find the reported artifact in the database and mark it as reported
    if ((static_cast<uint32_t>(res.artifact().type()) == static_cast<uint32_t>(artifacts[i].GetType())) &&
      artifacts[i].IsSameLocation(location))
    {
      ROS_INFO_STREAM("ArtifactReporter::BaseStationCB: Marking artifact " << i << " as reported");
      this->artifactDatabase.MarkArtifactAsReported(i);
      return;
    }
  }
}

void ArtifactReporter::BroadcastAvailableArtifacts()
{
  auto all_artifacts = this->artifactDatabase.GetArtifacts();
  uint32_t num_available = 0;

  for (const auto & artifact : all_artifacts)
  {
    if (!artifact.IsReported())
    {
      num_available++;
    }
  }

  std_msgs::UInt32 msg;
  msg.data = num_available;
  this->unreported_artifacts_pub.publish(msg);
}

bool ArtifactReporter::ReportServiceCB(
  artifact_localization_msgs::ReportArtifacts::Request & req,
  artifact_localization_msgs::ReportArtifacts::Response & resp
)
{
  this->ReportArtifacts();
  return true;
}
