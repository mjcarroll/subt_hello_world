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

#include "artifact_localization/TransformGenerator.hh"

geometry_msgs::TransformStamped GenerateTransform(
  const geometry_msgs::PoseStamped& offset,
  const std::string& parent_frame,
  const std::string& child_frame)
{
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = offset.header.stamp;
  transformStamped.header.frame_id = parent_frame;
  transformStamped.child_frame_id = child_frame;
  transformStamped.transform.translation.x = offset.pose.position.x;
  transformStamped.transform.translation.y = offset.pose.position.y;
  transformStamped.transform.translation.z = offset.pose.position.z;
  transformStamped.transform.rotation.x = offset.pose.orientation.x;
  transformStamped.transform.rotation.y = offset.pose.orientation.y;
  transformStamped.transform.rotation.z = offset.pose.orientation.z;
  transformStamped.transform.rotation.w = offset.pose.orientation.w;
  return transformStamped;
}
