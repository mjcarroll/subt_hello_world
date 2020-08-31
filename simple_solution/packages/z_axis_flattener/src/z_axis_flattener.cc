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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <z_axis_flattener/z_axis_flattener.hh>

ZAxisFlattener::ZAxisFlattener():
  tfListener_(this->tfBuffer_)
{
  std::string nodeName = ros::this_node::getName();
  this->nh_.param<std::string>(nodeName + "/flat_frame", this->flat_frame_, "map");
  this->nh_.param<std::string>(nodeName + "/name", this->name_, "X1");
  this->nh_.param<std::string>(nodeName + "/new_name", this->new_name_, "fake");
  ROS_INFO("TF Flatener Ready");

  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped outTf, inTf;

  ros::Rate rate(10.0);
  while (this->nh_.ok()){

    try{
      std::string parent = this->name_ + "/" + this->flat_frame_;
      std::string child = this->name_ + "/" + this->new_name_;
      inTf = this->tfBuffer_.lookupTransform(parent , this->name_, ros::Time(0));

      outTf.header.stamp = ros::Time::now();
      outTf.header.frame_id = parent;
      outTf.child_frame_id = child;

      tf2::Quaternion q, q0;
      tf2::fromMsg(inTf.transform.rotation, q);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      q0.setRPY(0, 0, yaw);
      outTf.transform.rotation = tf2::toMsg(q0);
      outTf.transform.translation.x = inTf.transform.translation.x;
      outTf.transform.translation.y = inTf.transform.translation.y;
      outTf.transform.translation.z = 0.0;

      br.sendTransform(outTf);
      rate.sleep();
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
};
