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

#ifndef UGV_FILTER_UGV_FILTER_H_
#define UGV_FILTER_UGV_FILTER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

/// \class UGVFilter UGVFilter.hh
/// \brief Class for flattening sloppes in PointCloud2 messages
class UGVFilter
{
  public:
    /// \brief Contstructor
    UGVFilter();

  private:
    /// \brief Re-publisher of ros sensor_msgs::PointCloud2
    /// Re-publishes the sensor_msgs::PointCloud2 once the slopes
    /// have been flattened
    /// \param[in] pc The sensor_msgs::PointCloud2 to flatten.
    void filterPC2(const sensor_msgs::PointCloud2::ConstPtr & pc);

    /// \brief ROS nodehandle. Used for subscribing to the PointCloud2 messages.
    /// Also used for publishing the modified message.
    ros::NodeHandle nh_;

    /// \brief Subscriber for incoming sensor_msgs::PointCloud2 message.
    ros::Subscriber pc2Sub_;

    /// \brief Publisher for incoming sensor_msgs::PointCloud2 message.
    /// Used to republish the modified PointCloud2 message.
    ros::Publisher pc2Pub_;

    /// \brief Buffer for transform between the pointcloud frame and the frame to flat to.
    tf2_ros::Buffer tfBuffer_;

    /// \brief Transform listener. Initializes tfBuffer.
    tf2_ros::TransformListener tfListener_;

    /// \brief Robot's name
    std::string name_;

    /// \brief Robot's new frame name
    std::string new_name_;

    /// \brief Frame to flat relative to it
    std::string flat_frame_;

    /// \brief Z value to flatten to.
    float flat_height_;

    /// \brief Max slope value to filter from.
    float slope_threshold_;

    /// \brief Boolean representing if points are going to be flattened or deleted
    bool delete_;
};

#endif
