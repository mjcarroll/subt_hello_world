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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_fix/fix_camera.hh>


FixingNode::FixingNode():
  imgSub_(this->nh_.subscribe("input/image_raw", 1, &FixingNode::fixImgMsg, this)),
  camSub_(this->nh_.subscribe("input/camera_info", 1, &FixingNode::fixCamMsg, this)),
  imgPub_(this->nh_.advertise<sensor_msgs::Image>("output/image_raw", 1)),
  camPub_(this->nh_.advertise<sensor_msgs::CameraInfo>("output/camera_info", 1))
{
  std::string nodeName = ros::this_node::getName();
  if (!this->nh_.getParam(nodeName + "/new_frame_id", this->new_frame_id_)){
    ROS_WARN_STREAM("No parameter at " << nodeName << "/new_frame_id"
     << " defaulting to" << nodeName <<"/base_link/camera_frame");
    this->new_frame_id_ = "/base_link/camera_frame";
  }
  ROS_INFO("Camera Relay Ready");
};

void FixingNode::fixImgMsg(const sensor_msgs::Image::Ptr & img){
  img->header.frame_id = new_frame_id_;
  this->imgPub_.publish(img);
};

void FixingNode::fixCamMsg(const sensor_msgs::CameraInfo::Ptr & cam){
  cam->header.frame_id = new_frame_id_;
  this->camPub_.publish(cam);
};
