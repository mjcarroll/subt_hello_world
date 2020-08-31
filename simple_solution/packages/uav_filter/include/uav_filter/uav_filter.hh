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

#ifndef UAV_FILTER_UAV_FILTER_H_
#define UAV_FILTER_UAV_FILTER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

/// \class UAVFilter uav_filter.hh
/// \brief Class for filtering the uav and anything below its legs from the pointcloud
class UAVFilter
{
  public:
    /// \brief Contstructor
    UAVFilter();

  private:
    /// \brief Re-publisher of ros sensor_msgs::PointCloud2
    /// Publishes the sensor_msgs::PointCloud2 once the drone
    /// volume and the obstacles that are not high enough to
    /// modify the flight path has been cropped out.
    /// \param[in] pc The sensor_msgs::PointCloud2 to filter.
    void filterPC2(const sensor_msgs::PointCloud2::Ptr & pc);

    /// \brief ROS nodehandle. Used for subscribing to the PointCloud2 messages.
    /// Also used for publishing the modified message.
    ros::NodeHandle nh_;

    /// \brief Subscriber for incoming sensor_msgs::PointCloud2 message.
    ros::Subscriber pc2Sub_;

    /// \brief Publisher for incoming sensor_msgs::PointCloud2 message.
    /// Used to publish the modified PointCloud2 message.
    ros::Publisher pc2Pub_;

    /// \brief maximun value of cordinate x to crop from.
    float x_max_;

    /// \brief minimun value of cordinate x to crop from.
    float x_min_;

    /// \brief maximun value of cordinate y to crop from.
    float y_max_;

    /// \brief minimun value of cordinate y to crop from.
    float y_min_;

    /// \brief maximun value of cordinate z to crop from.
    float z_max_;

    /// \brief minimun value of cordinate z to crop from.
    float z_min_;

    /// \brief The name of the map frame.
    std::string map_frame_;

    /// \brief Buffer for transform listener that finds the transform between the map origin and the robot frame.
    tf2_ros::Buffer tfBuffer_;

    /// \brief Transform listener. Initializes tfBuffer.
    tf2_ros::TransformListener tfListener_;
};
#endif
