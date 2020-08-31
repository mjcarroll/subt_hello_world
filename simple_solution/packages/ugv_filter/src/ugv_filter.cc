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

#include <ros/ros.h>
#include <ugv_filter/ugv_filter.hh>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Point.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


UGVFilter::UGVFilter():
  tfListener_(this->tfBuffer_),
  pc2Sub_(this->nh_.subscribe("input", 1, &UGVFilter::filterPC2, this)),
  pc2Pub_(this->nh_.advertise<sensor_msgs::PointCloud2>("output", 1))
{
  std::string nodeName = ros::this_node::getName();
  this->nh_.param<float>(nodeName + "/flat_height", this->flat_height_, -0.25);
  this->nh_.param<std::string>(nodeName + "/flat_frame", this->flat_frame_, "map");
  this->nh_.param<std::string>(nodeName + "/name", this->name_, "X1");
  this->nh_.param<std::string>(nodeName + "/new_name", this->new_name_, "fake");
  this->nh_.param<bool>(nodeName + "/delete", this->delete_, true);
  this->nh_.param<float>(nodeName + "/slope", this->slope_threshold_, 45);
  ROS_INFO("UGV Filter Ready");
};

void UGVFilter::filterPC2(const sensor_msgs::PointCloud2::ConstPtr & pc){
  pcl::PointCloud<pcl::PointXYZ>::Ptr body(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr publishBody(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 result;

  geometry_msgs::PointStamped origin;
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = this->tfBuffer_.lookupTransform(pc->header.frame_id,
      this->name_ + "/" + this->flat_frame_, ros::Time(0), ros::Duration(5.0));
    geometry_msgs::PointStamped tempOrigin;
    tempOrigin.point.x = 0;
    tempOrigin.point.y = 0;
    tempOrigin.point.z = this->flat_height_;
    tempOrigin.header = transformStamped.header;
    tf2::doTransform(tempOrigin, origin, transformStamped);
    tf2::doTransform(*pc, result, transformStamped);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }

  pcl::fromROSMsg(result, *body);
  pcl::fromROSMsg(*pc, *publishBody);
  for (size_t x = 0; x < body->width; x++) {
    geometry_msgs::PointStamped last_point;
    last_point.point.x = 0;
    last_point.point.y = 0;
    last_point.point.z = 0;
    for (size_t y = 0; y < body->height; y++) {
      geometry_msgs::PointStamped transformed;
      pcl::PointXYZ actual_point = body->at(x,y);
      float diff_z = actual_point.z - last_point.point.z;
      float diff_x = actual_point.x - last_point.point.x;
      float diff_y = actual_point.y - last_point.point.y;
      float slope = atan2(diff_z, sqrt(diff_x * diff_x + diff_y * diff_y)) * 180 / M_PI;
      last_point.point.x = actual_point.x;
      last_point.point.y = actual_point.y;
      last_point.point.z = actual_point.z;
      if (slope == NAN || slope == INFINITY) {
        continue;
      }
      if (slope < this->slope_threshold_ && slope > -this->slope_threshold_) {
        if (this->delete_) {
          actual_point.x = INFINITY;
          actual_point.y = INFINITY;
        }
        actual_point.z = origin.point.z;
        publishBody->at(x,y) = actual_point;
      }
      else
      {
        break;
      }
    }
  }
  pcl::toROSMsg(*publishBody, result);
  result.header.frame_id = this->name_ + "/" + this->new_name_ +
    "/" + "sensor_" + this->new_name_;
  this->pc2Pub_.publish(result);
};
