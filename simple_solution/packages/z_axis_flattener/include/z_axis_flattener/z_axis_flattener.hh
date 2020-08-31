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

#ifndef Z_AXIS_FLATTENER_Z_AXIS_FLATTENER_HH_
#define Z_AXIS_FLATTENER_Z_AXIS_FLATTENER_HH_

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

/// \class ZAxisFlattener ZAxisFlattener.hh
/// \brief Class for creating a flattened frame for move_base to properly work.
class ZAxisFlattener
{
  public:
    /// \brief Contstructor
    ZAxisFlattener();

  private:
    /// \brief ROS nodehandle.
    ros::NodeHandle nh_;

    /// \brief Buffer for transform between the robot's frame and the frame to flat to.
    tf2_ros::Buffer tfBuffer_;

    /// \brief Transform listener. Initializes tfBuffer.
    tf2_ros::TransformListener tfListener_;

    /// \brief Robot's name
    std::string name_;

    /// \brief Frame to flat relative to it
    std::string flat_frame_;

    /// \brief Name for the new frame
    std::string new_name_;
};

#endif
