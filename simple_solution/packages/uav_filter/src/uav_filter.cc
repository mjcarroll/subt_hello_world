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
#include <uav_filter/uav_filter.hh>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


UAVFilter::UAVFilter():
  tfListener_(this->tfBuffer_),
  pc2Sub_(this->nh_.subscribe("input", 1, &UAVFilter::filterPC2, this)),
  pc2Pub_(this->nh_.advertise<sensor_msgs::PointCloud2>("output", 1))
{
  std::string nodeName = ros::this_node::getName();
  std::string robot_name;
  this->nh_.param<std::string>(nodeName + "/map_frame", this->map_frame_, "map");
  this->nh_.param<std::string>(nodeName + "/name", robot_name, "X1");
  this->nh_.param<float>(nodeName + "/x_max", this->x_max_, 0.3);
  this->nh_.param<float>(nodeName + "/x_min", this->x_min_, -0.3);
  this->nh_.param<float>(nodeName + "/y_max", this->y_max_, 0.3);
  this->nh_.param<float>(nodeName + "/y_min", this->y_min_, -0.3);
  this->nh_.param<float>(nodeName + "/z_max", this->z_max_, -0.3);
  this->nh_.param<float>(nodeName + "/z_min", this->z_min_, 0.3);
  this->map_frame_ = robot_name + "/" + this->map_frame_;
  ROS_INFO("UAV Filter Ready");
};

void UAVFilter::filterPC2(const sensor_msgs::PointCloud2::Ptr & pc){
  pcl::PointCloud<pcl::PointXYZ>::Ptr body(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pc, *body);
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  boxFilter.setKeepOrganized(true);
  boxFilter.setMax(Eigen::Vector4f(this->x_max_, this->y_max_, this->z_max_, 1.0));
  boxFilter.setMin(Eigen::Vector4f(this->x_min_, this->y_min_, this->z_min_, 1.0));
  boxFilter.setNegative(true);
  boxFilter.setInputCloud(body);
  boxFilter.filter(*bodyFiltered);
  pcl::toROSMsg(*bodyFiltered, *pc);

  geometry_msgs::PointStamped origin;
  sensor_msgs::PointCloud2 transformed_pc;
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::TransformStamped transformStampedBack;
  try{
    transformStamped = this->tfBuffer_.lookupTransform(map_frame_, pc->header.frame_id, ros::Time(0), ros::Duration(5.0));
    transformStampedBack = this->tfBuffer_.lookupTransform(pc->header.frame_id, map_frame_, ros::Time(0), ros::Duration(5.0));
    tf2::doTransform(*pc, transformed_pc, transformStamped);
    geometry_msgs::PointStamped tempOrigin;
    tempOrigin.point.x = 0;
    tempOrigin.point.y = 0;
    tempOrigin.point.z = this->z_min_;
    tempOrigin.header = transformStamped.header;
    tf2::doTransform(tempOrigin, origin, transformStamped);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    this->pc2Pub_.publish(pc);
    return;
  }
  pcl::fromROSMsg(transformed_pc, *body);
  pcl::PassThrough<pcl::PointXYZ> filter;
  filter.setKeepOrganized(true);
  filter.setFilterFieldName("z");
  filter.setFilterLimits(-300.0, origin.point.z);
  filter.setFilterLimitsNegative(true);
  filter.setInputCloud(body);
  filter.filter(*bodyFiltered);

  pcl::toROSMsg(*bodyFiltered, *pc);
  try{
    tf2::doTransform(*pc, *pc, transformStampedBack);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }
  this->pc2Pub_.publish(pc);
};
