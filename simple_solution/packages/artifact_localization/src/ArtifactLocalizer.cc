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

#include "artifact_localization/ArtifactLocalizer.hh"
#include "artifact_localization/TransformGenerator.hh"
#include "darknet_ros_msgs/BoundingBox.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/common/centroid.h"
#include "subt_ign/CommonTypes.hh"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/CameraInfo.h>
//DEBUG
#include "visualization_msgs/Marker.h"

ArtifactLocalizer::ArtifactLocalizer():
  camera_params_ready_(false),
  tfListener(this->tfBuffer),
  ci_sub(this->nh.subscribe("input/camera_info", 1, &ArtifactLocalizer::ReceiveCameraParams, this)),
  pc_sub(this->nh, "input/point_cloud", 1),
  bb_sub(this->nh, "input/bounding_boxes", 1),
  sync(ApproximationSyncPolicy(20), this->pc_sub, this->bb_sub)
{
  std::string nodeName = ros::this_node::getName();
  this->nh.param<std::string>(nodeName + "/camera_frame", this->camera_frame_, "X1/base_link/camera_front");
  this->nh.param<std::string>(nodeName + "/object_frame", this->object_frame_, "object_frame");
  this->nh.param<std::string>(nodeName + "/artifact_origin_frame", this->artifact_origin_frame_, "artifact_origin");
  this->nh.param<float>(nodeName + "/external_robot_tolerance", this->external_robot_tolerance_, 3.0);

  pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_pc", 1);
  sync.registerCallback(boost::bind(&ArtifactLocalizer::ProcessDetection, this, _1, _2));
}

void ArtifactLocalizer::ReceiveCameraParams(const sensor_msgs::CameraInfo::ConstPtr & camInfo)
{
  this->cx_ = camInfo->P[2];
  this->fx_ = camInfo->P[0];
  this->cy_ = camInfo->P[6];
  this->fy_ = camInfo->P[5];
  this->camera_params_ready_ = true;
}

void ArtifactLocalizer::ProcessDetection(
  const sensor_msgs::PointCloud2::ConstPtr & cloud_msg,
  const darknet_ros_msgs::BoundingBoxes::ConstPtr & bb_msg)
{
  if (!this->camera_params_ready_)
  {
    ROS_INFO("Cameras Parameters not set");
    return;
  }
  // create a new PointCloud2 msg that will only contain the points in the bounding box
  sensor_msgs::PointCloud2 cropped_cloud_msg;
  cropped_cloud_msg.header = cloud_msg->header;
  cropped_cloud_msg.fields = cloud_msg->fields;
  cropped_cloud_msg.is_bigendian = cloud_msg->is_bigendian;
  cropped_cloud_msg.point_step = cloud_msg->point_step;
  cropped_cloud_msg.is_dense = cloud_msg->is_dense;

  for (const auto & box : bb_msg->bounding_boxes)
  {
    auto typeInfo = this->ValidateType(box.Class);
    if (!typeInfo.first)
    {
      continue;
    }

    this->CropPointCloud(cloud_msg, cropped_cloud_msg, box, bb_msg->image_header.frame_id);
    this->pub.publish(cropped_cloud_msg);

    if (cropped_cloud_msg.data.size() <= 0) {
      return;
    }
    auto centroid = this->GetCentroid(cropped_cloud_msg);
    auto p = this->ToPoseStamped(centroid);
    auto tf = GenerateTransform(
      p, cloud_msg->header.frame_id, object_frame_);
    this->tfBroadcaster.sendTransform(tf);

    this->SaveDetectionForReporting(p, box.probability, typeInfo.second);
  }
}

std::pair<bool, subt::ArtifactType> ArtifactLocalizer::ValidateType(const std::string & predictedType) const
{
  std::pair<bool, subt::ArtifactType> result;
  result.first = false;

  if ((predictedType == "backpack") || (predictedType == "suitcase"))
  {
    result.first = true;
    result.second = subt::ArtifactType::TYPE_BACKPACK;
  }
  else if (predictedType == "person")
  {
    result.first = true;
    result.second = subt::ArtifactType::TYPE_RESCUE_RANDY;
  }
  else if (predictedType == "rope")
  {
    result.first = true;
    result.second = subt::ArtifactType::TYPE_ROPE;
  }
  else if (predictedType == "helmet")
  {
    result.first = true;
    result.second = subt::ArtifactType::TYPE_HELMET;
  }


  return result;
}

void ArtifactLocalizer::CropPointCloud(
  const sensor_msgs::PointCloud2::ConstPtr & original_pc,
  sensor_msgs::PointCloud2 & modified_pc,
  const darknet_ros_msgs::BoundingBox & bb, const std::string image_frame) const
{
  // as darknet has an inverted frame, take max as min and min as max
  float yMax = (bb.ymax - this->cy_) / this->fy_;
  float yMin = (bb.ymin - this->cy_) / this->fy_;
  float xMax = (bb.xmax - this->cx_) / this->fx_;
  float xMin = (bb.xmin - this->cx_) / this->fx_;;
  // The original poincloud gets cropped from the projection of the bounding box.
  // That projecion represents a square-based pyramid, with p0 being the apex and
  // p1-p4 being the vertices of the base, and max_detection the length of sides.
  geometry_msgs::PointStamped p0, p1, p2, p3, p4;
  geometry_msgs::TransformStamped transformStamped;
  float max_detection = 20;

  try{
    transformStamped = this->tfBuffer.lookupTransform(original_pc->header.frame_id, image_frame, ros::Time(0), ros::Duration(5.0));
    geometry_msgs::PointStamped temp0, temp1, temp2, temp3, temp4;
    temp1.point.x = xMax * max_detection;
    temp1.point.y = yMax * max_detection;
    temp1.point.z = max_detection;
    temp1.header = transformStamped.header;
    temp2.point.x = xMax * max_detection;
    temp2.point.y = yMin * max_detection;
    temp2.point.z = max_detection;
    temp2.header = transformStamped.header;
    temp3.point.x = xMin * max_detection;
    temp3.point.y = yMin * max_detection;
    temp3.point.z = max_detection;
    temp3.header = transformStamped.header;
    temp4.point.x = xMin * max_detection;
    temp4.point.y = yMax * max_detection;
    temp4.point.z = max_detection;
    temp4.header = transformStamped.header;
    temp0.point.x = 0;
    temp0.point.y = 0;
    temp0.point.z = 0;
    temp0.header = transformStamped.header;
    tf2::doTransform(temp0, p0, transformStamped);
    tf2::doTransform(temp1, p1, transformStamped);
    tf2::doTransform(temp2, p2, transformStamped);
    tf2::doTransform(temp3, p3, transformStamped);
    tf2::doTransform(temp4, p4, transformStamped);
  }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr body(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*original_pc, *body);
  pcl::ExtractIndices<pcl::PointXYZ> pyramidFilter;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pyramidFilter.setInputCloud(body);
  tf2::Vector3 plane1, plane2, plane3, plane4;
  tf2::Vector3 p0v(p0.point.x, p0.point.y, p0.point.z);
  tf2::Vector3 p1v(p1.point.x, p1.point.y, p1.point.z);
  tf2::Vector3 p2v(p2.point.x, p2.point.y, p2.point.z);
  tf2::Vector3 p3v(p3.point.x, p3.point.y, p3.point.z);
  tf2::Vector3 p4v(p4.point.x, p4.point.y, p4.point.z);
  // To crop the points inside the pyramid, we calculate the 4 planes of the pyramid
  // faces, then select the points that give negative distance to all those planes
  // (that would mean that the point is inside all 4 faces, ergo, inside the pyramid)
  float plane1d, plane2d, plane3d, plane4d;
  plane1 = tf2::tf2Cross(p1v - p0v, p2v - p0v);
  plane1d = tf2::tf2Dot(plane1, p0v);
  plane2 = tf2::tf2Cross(p2v - p0v, p3v - p0v);
  plane2d = tf2::tf2Dot(plane2, p0v);
  plane3 = tf2::tf2Cross(p3v - p0v, p4v - p0v);
  plane3d = tf2::tf2Dot(plane3, p0v);
  plane4 = tf2::tf2Cross(p4v - p0v, p1v - p0v);
  plane4d = tf2::tf2Dot(plane4, p0v);

  for (size_t i = 0; i < body->size(); i++) {
    tf2::Vector3 temp(body->points[i].x, body->points[i].y, body->points[i].z);
    float dist1 = tf2::tf2Dot(temp, plane1) - plane1d;
    float dist2 = tf2::tf2Dot(temp, plane2) - plane2d;
    float dist3 = tf2::tf2Dot(temp, plane3) - plane3d;
    float dist4 = tf2::tf2Dot(temp, plane4) - plane4d;
    if (dist1 < 0 && dist2 < 0 && dist3 < 0 && dist4 < 0) {
      inliers->indices.push_back(i);
    }
  }
  pyramidFilter.setIndices(inliers);
  pyramidFilter.filter(*bodyFiltered);
  pcl::toROSMsg(*bodyFiltered, modified_pc);
}

pcl::PointXYZ ArtifactLocalizer::GetCentroid(sensor_msgs::PointCloud2 & cloud_msg) const
{
  // Convert to PCL data type
  auto cloud = std::make_shared<pcl::PCLPointCloud2>();
  pcl_conversions::toPCL(cloud_msg, *cloud);
  pcl::PointCloud<pcl::PointXYZ> pclObj;
  pcl::fromPCLPointCloud2(*cloud, pclObj);

  // Save all points in the point cloud to a centroid object
  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  for (auto &p : pclObj.points)
  {
    if (isinf(p.x) || isinf(p.y) || isinf(p.z))
    {
      continue;
    }
    centroid.add(p);
  }

  // extract the centroid
  pcl::PointXYZ location;
  centroid.get(location);

  return location;
}

geometry_msgs::PoseStamped ArtifactLocalizer::ToPoseStamped(const pcl::PointXYZ & objCentroid) const
{
  geometry_msgs::PoseStamped p;
  p.header.frame_id = camera_frame_;
  p.header.stamp = ros::Time::now();
  p.pose.position.x = objCentroid.x;
  p.pose.position.y = objCentroid.y;
  p.pose.position.z = objCentroid.z;
  p.pose.orientation.x = 0.0;
  p.pose.orientation.y = 0.0;
  p.pose.orientation.z = 0.0;
  p.pose.orientation.w = 1.0;

  return p;
}

void ArtifactLocalizer::SaveDetectionForReporting(
  const geometry_msgs::PoseStamped & objPose,
  const double confidence,
  const subt::ArtifactType & type)
{
  try
  {
    auto scoring_pose = this->tfBuffer.transform<geometry_msgs::PoseStamped>(
      objPose, artifact_origin_frame_, ros::Duration(1.0));
    ROS_INFO_STREAM(artifact_origin_frame_ << " -> object is " <<
      scoring_pose.pose.position.x << ","
      << scoring_pose.pose.position.y << ","
      << scoring_pose.pose.position.z);

    geometry_msgs::Point location;
    location.x = scoring_pose.pose.position.x;
    location.y = scoring_pose.pose.position.y;
    location.z = scoring_pose.pose.position.z;
    if (!this->robot_radar_.CheckPointWithinRadar(location, this->external_robot_tolerance_))
    {
      Detection detection(location, confidence, type);
      this->artifactReporter.AddDetectionToDB(detection);
    }
  }
  catch(tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}
