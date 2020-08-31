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

#include <subt_robot_control/controller.hh>

#include <functional>

#include <artifact_localization_msgs/ReportArtifacts.h>
#include <nav_msgs/Odometry.h>
#include <periodic_data_saver/data_saver.hh>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/SetBool.h>
#include <tf2_msgs/TFMessage.h>

Controller::Controller() :
  private_nh_("~"),
  artifact_report_client_(this->nh_.serviceClient<artifact_localization_msgs::ReportArtifacts>("report_artifacts")),
  back_track_state_(false),
  tf_listener_(tf_buffer_),
  pose_publisher_(this->nh_.advertise<std_msgs::String>("local_robot_pose", 10))
{
  std::string map_frame;
  this->private_nh_.param<std::string>("artifact_origin_frame", this->artifact_origin_frame_, "artifact_origin");
  this->private_nh_.param<bool>("has_odom", this->has_odom_, true);
  this->private_nh_.param<std::string>("map_frame", map_frame, "map");
  this->private_nh_.param<float>("min_battery_percentage", this->min_battery_percentage_, 0.35);
  this->private_nh_.param<std::string>("name", this->robot_name_, "X1");

  this->PerformStartAction();

  // Wait for artifact_origin transformation to be published
  this->WaitForOrigin(map_frame);

  this->battery_sub_ = this->nh_.subscribe("battery_state", 1, &Controller::BatteryCallback, this);

  this->comms_manager_ = new CommsManager();
  this->pose_timer_ = this->nh_.createTimer(ros::Duration(0.2), &Controller::SharePose, this);

  this->graph_controller_ = new graph_controller::Controller(this->robot_name_,
                                                             this->artifact_origin_frame_,
                                                             std::bind(&Controller::BacktrackStarted, this));

  // start the backtrack routine after 40 minutes if it hasn't started yet
  this->backtrack_timer_ = this->nh_.createTimer(ros::Duration(2400),
                                                 [this](const ros::TimerEvent&)
                                                 {
                                                   if (!this->back_track_state_)
                                                   {
                                                     ROS_INFO("Controller's backtrack timer called; backtracking");
                                                     this->back_track_state_ = true;
                                                     this->graph_controller_->Backtrack();
                                                   }
                                                 },
                                                 true);
}

void Controller::PerformStartAction()
{
  ROS_INFO("Control node: beginning the start action...");

  ROS_DEBUG("Waiting for services...");
  ros::service::waitForService("/subt/start");
  ros::service::waitForService("/subt/finish");
  ros::service::waitForService("/subt/pose_from_artifact_origin");

  boost::shared_ptr<rosgraph_msgs::Clock const> clock = ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", this->nh_);
  ROS_DEBUG("All Simulation services found: %ld",  std::time(nullptr));
  ROS_DEBUG("Simulation clock: %d.%d", clock->clock.sec, clock->clock.nsec);

  ROS_DEBUG("Waiting for topics...");
  bool found_all_topics = false;
  while (!found_all_topics)
  {
    bool have_odom = true;
    if(this->has_odom_) {
      have_odom = !!ros::topic::waitForMessage<nav_msgs::Odometry>("odom", this->nh_, ros::Duration(5.0));
    }
    auto have_imu = !!ros::topic::waitForMessage<sensor_msgs::Imu>("imu/data", this->nh_, ros::Duration(5.0));
    auto have_battery = !!ros::topic::waitForMessage<sensor_msgs::BatteryState>("battery_state", this->nh_, ros::Duration(5.0));
    auto have_pose = !!ros::topic::waitForMessage<tf2_msgs::TFMessage>("pose", this->nh_, ros::Duration(5.0));
    auto have_pc = !!ros::topic::waitForMessage<sensor_msgs::PointCloud2>("points", this->nh_, ros::Duration(5.0));
    auto have_img = !!ros::topic::waitForMessage<sensor_msgs::Image>("front/image_raw", this->nh_, ros::Duration(5.0));

    found_all_topics = have_odom && have_imu && have_battery && have_pose && have_pc && have_img;
  }

  ROS_DEBUG("All Robot topics found");

  ros::ServiceClient serviceClient=this->nh_.serviceClient<std_srvs::SetBool>("/subt/start");
  std_srvs::SetBool request;
  request.request.data= true;
  if(!serviceClient.call(request))
  {
    ROS_ERROR("Error calling /subt/start service");
    ros::shutdown();
  }
  ROS_INFO("Control node: Start action complete; simulation started");
}

void Controller::PerformStopAction()
{
  ROS_INFO("Control node: beginning the stop action...");

  // publish the last map and trajectory list messages that were generated
  periodic_data_saver::DataSaver data_saver(false);
  data_saver.SaveData();

  // pause for a moment to make sure the published messages above were saved to the rosbag before shutting down
  ros::Duration(20.0).sleep();

  // report the artifacts that were found
  artifact_localization_msgs::ReportArtifacts srv;
  if (!this->artifact_report_client_.call(srv))
  {
    ROS_ERROR("Report artifacts service call FAILED");
  }

  ROS_INFO("Control node: stop action done");
}

void Controller::BatteryCallback(const sensor_msgs::BatteryState & msg)
{
  auto battery_percentage = msg.percentage;
  if (battery_percentage < this->min_battery_percentage_ &&
      !this->back_track_state_)
  {
    ROS_INFO_STREAM("Battery percentage ("
                    << battery_percentage
                    << ") is lower than min_battery_percentage ("
                    << this->min_battery_percentage_
                    << "). Starting to backtrack now");
    this->back_track_state_ = true;
    this->graph_controller_->Backtrack();
  }
  else if (this->back_track_state_ &&
           this->graph_controller_->DoneBacktracking())
  {
    this->PerformStopAction();
  }
}

void Controller::WaitForOrigin(const std::string & map_frame)
{
  const std::string full_map_frame = this->robot_name_ + "/" + map_frame;

  bool tf_ready = false;
  while (!tf_ready)
  {
    tf_ready = this->tf_buffer_.canTransform(this->artifact_origin_frame_,
                                             full_map_frame,
                                             ros::Time::now(),
                                             ros::Duration(10.0));
  }

  ROS_DEBUG("TF tree is ready to be used");
}

void Controller::SharePose(const ros::TimerEvent&)
{
  if (!this->tf_buffer_.canTransform(this->artifact_origin_frame_,
                                     this->robot_name_,
                                     ros::Time(0)))
  {
    return;
  }

  auto pose_tf = this->tf_buffer_.lookupTransform(this->artifact_origin_frame_,
                                                  this->robot_name_,
                                                  ros::Time(0));

  ROS_DEBUG_STREAM("Controller::SharePose: sharing a new pose (x,y,z): "
                  << pose_tf.transform.translation.x << " "
                  << pose_tf.transform.translation.y << " "
                  << pose_tf.transform.translation.z);

  ignition::msgs::Pose pose;
  pose.mutable_position()->set_x(pose_tf.transform.translation.x);
  pose.mutable_position()->set_y(pose_tf.transform.translation.y);
  pose.mutable_position()->set_z(pose_tf.transform.translation.z);
  pose.set_name(this->robot_name_);

  std::string serialized_pose;
  if (!pose.SerializeToString(&serialized_pose))
  {
    ROS_ERROR_STREAM("Controller::SharePose: Error serializing pose\n" << pose.DebugString());
    return;
  }

  std_msgs::String msg;
  msg.data = serialized_pose;
  this->pose_publisher_.publish(msg);
}

void Controller::BacktrackStarted()
{
  this->back_track_state_ = true;
}
