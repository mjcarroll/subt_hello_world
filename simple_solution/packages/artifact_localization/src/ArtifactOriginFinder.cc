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

#include "artifact_localization/ArtifactOriginFinder.hh"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ignition/math/Pose3.hh"
#include "artifact_localization/TransformGenerator.hh"

ArtifactOriginFinder::ArtifactOriginFinder() :
  serviceName("/subt/pose_from_artifact_origin"),
  tfListener(this->tfBuffer)
{

  std::string nodeName = ros::this_node::getName();
  this->nh.param<std::string>(nodeName + "/map_frame", this->map_frame_, "X1/map");
  this->nh.param<std::string>(nodeName + "/artifact_origin_frame", this->artifact_origin_frame_, "artifact_origin");
  this->nh.param<std::string>(nodeName + "/base_link_frame", this->base_link_frame_, "X1");
  this->nh.param<std::string>(nodeName + "/name", this->robot_name_, "X1");

  this->FindArtifactOrigin();

  // map->artifact_origin tf will be broadcasted periodically through a timer
  this->timer = nh.createTimer(ros::Duration(2.0),
    boost::bind(&ArtifactOriginFinder::BroadcastArtifactOriginTF, this));
}

void ArtifactOriginFinder::FindArtifactOrigin()
{
  // set up service call
  ros::ServiceClient client = this->nh.serviceClient<subt_msgs::PoseFromArtifact>(this->serviceName);
  subt_msgs::PoseFromArtifact srv;
  srv.request.robot_name.data = robot_name_;

  do
  {
    ROS_INFO("Waiting for map -> base_link tf to become available...");
  } while (!tfBuffer.canTransform(map_frame_,
    base_link_frame_, ros::Time(0), ros::Duration(5.0)));

  // call service and save transforms
  if (client.call(srv))
  {
    // save static transfrom between artifact_origin->map
    try
    {
      geometry_msgs::TransformStamped base2map = tfBuffer.lookupTransform(this->base_link_frame_, this->map_frame_, ros::Time(0), ros::Duration(5.0));
      geometry_msgs::TransformStamped origin2base = GenerateTransform(srv.response.pose,
        this->artifact_origin_frame_, this->map_frame_);

      origin2base.transform.translation.x += base2map.transform.translation.x;
      origin2base.transform.translation.y += base2map.transform.translation.y;
      origin2base.transform.translation.z += base2map.transform.translation.z;
      this->transformStamped = origin2base;
    }
    catch(tf2::TransformException &ex)
    {
      ROS_FATAL("%s", ex.what());
      ros::shutdown();
    }
  }
  else
  {
    ROS_FATAL_STREAM("service call failed: " << this->serviceName);
    ros::shutdown();
  }
}

void ArtifactOriginFinder::BroadcastArtifactOriginTF()
{
  this->staticTfBroadcaster.sendTransform(this->transformStamped);
}
