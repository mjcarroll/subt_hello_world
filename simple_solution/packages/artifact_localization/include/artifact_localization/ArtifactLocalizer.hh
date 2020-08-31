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

#ifndef ARTIFACT_LOCALIZATION_ARTIFACT_LOCALIZER_H_
#define ARTIFACT_LOCALIZATION_ARTIFACT_LOCALIZER_H_

#include "ArtifactReporter.hh"
#include "darknet_ros_msgs/BoundingBox.h"
#include "ros/ros.h"
#include "subt_ign/CommonTypes.hh"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "pcl/point_types.h"
#include "geometry_msgs/PoseStamped.h"
#include <utility>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <sensor_msgs/CameraInfo.h>
#include "message_filters/sync_policies/approximate_time.h"
#include <artifact_localization/ExternalPointRadar.hh>

/// \brief Approximate time policy for the message synchronizer,
/// which allows for syncing of the lidar and camera data even if the sensors
/// process data at slightly different times.
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes> ApproximationSyncPolicy;

/// \class ArtifactLocalizer ArtifactLocalizer.hh
/// \brief Class for finding and localizing artifacts relative to the artifact origin frame.
class ArtifactLocalizer
{
  /// \brief Constructor
  public: ArtifactLocalizer();

  /// \brief Takes in a synchronized point cloud and bounding box, and determines where the centroid
  /// of the bounding box is relative to the artifact origin frame.
  /// \param[in] cloud_msg The point cloud data associated with the bounding box data.
  /// \param[in] bb_msg The bounding box data that was generated by darknet.
  private: void ProcessDetection(
    const sensor_msgs::PointCloud2::ConstPtr & cloud_msg,
    const darknet_ros_msgs::BoundingBoxes::ConstPtr & bb_msg);

  /// \brief Checks to make sure that the object type detected by darknet is a valid artifact type.
  /// \param[in] predictedType The type of object predicted by darknet.
  /// \returns A pair where the first field indicates whether predictedType is valid or not. If the first field is
  /// true, the second field of this pair defines what type of artifact was predicted.
  private: std::pair<bool, subt::ArtifactType> ValidateType(const std::string & predictedType) const;

  /// \brief Receives the CameraInfo message and get camera configuration parameters from it.
  /// \param[in] sensor_msgs::CameraInfo message.
  private: void ReceiveCameraParams(const sensor_msgs::CameraInfo::ConstPtr & camInfo);

  /// \brief Crops point cloud data to only the points that are within a bounding box.
  /// \param[in] original_pc The original, uncropped point cloud data.
  /// \param[in] modified_pc The new, cropped point cloud data that will only contain points inside of bb.
  /// \param[in] bb The bounding box that defines which points are to be saved to modified_pc.
  private: void CropPointCloud(
    const sensor_msgs::PointCloud2::ConstPtr & original_pc,
    sensor_msgs::PointCloud2 & modified_pc,
    const darknet_ros_msgs::BoundingBox & bb,
    const std::string image_frame) const;

  /// \brief Gets the centroid of a point cloud.
  /// \param[in] cloud_msg The point cloud data to extract the centroid from.
  /// \returns The centroid of cloud_msg.
  private: pcl::PointXYZ GetCentroid(sensor_msgs::PointCloud2 & cloud_msg) const;

  /// \brief Converts a centroid to a PoseStamped msg.
  /// \returns The converted PoseStamped msg.
  private: geometry_msgs::PoseStamped ToPoseStamped(const pcl::PointXYZ & objCentroid) const;

  /// \brief Creates a detection and adds this detection to the database so that it can be reported later if necessary.
  /// The pose passed in is transformed with respect to the artifact origin frame before saving it as a detection.
  /// \param[in] objPose The pose of the object to be reported, relative to the camera frame at the time of detection.
  /// \param[in] confidence The confidence in the type of object detection.
  /// \param[in] type The type of object that was detected.
  private: void SaveDetectionForReporting(
    const geometry_msgs::PoseStamped & objPose,
    const double confidence,
    const subt::ArtifactType & type);

  /// \brief ROS nodehandle. Used for synchronizing subscriptions from the point cloud and bounding box topics.
  /// Also used for publishing the cropped point cloud to a topic.
  private: ros::NodeHandle nh;

  /// \brief Publisher for newly created cropped point clouds.
  private: ros::Publisher pub;

  /// \brief Subscriber for getting camera parameters from a CameraInfo message.
  private: ros::Subscriber ci_sub;

  /// \brief Transform broadcaster for newly detected objects.
  private: tf2_ros::TransformBroadcaster tfBroadcaster;

  /// \brief Buffer for transform listener that finds the transform between the artifact origin and newly detected objects.
  private: tf2_ros::Buffer tfBuffer;

  /// \brief Transform listener. Initializes tfBuffer.
  private: tf2_ros::TransformListener tfListener;

  /// \brief ArtifactReporter object that holds the database of found artifacts.
  /// Used to communicate with the base station and update the artifact database.
  private: ArtifactReporter artifactReporter;

  /// \brief Subscriber for incoming point cloud data.
  private: message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;

  /// \brief Subscriber for incoming bounding box data.
  private: message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb_sub;

  /// \brief Time synchronizer for incoming point cloud and bounding box data.
  private: message_filters::Synchronizer<ApproximationSyncPolicy> sync;

  /// \brief Flag to know if camera parameters have been read.
  private: bool camera_params_ready_;

  /// \brief Principal Point x value for projection.
  private: float cx_;

  /// \brief Focal length x value for projection.
  private: float fx_;

  /// \brief Principal Point y value for projection.
  private: float cy_;

  /// \brief Focal length y value for projection.
  private: float fy_;

  /// \brief The name of the camera frame.
  private:  std::string camera_frame_;

  /// \brief The name of the object frame.
  /// This is the frame that corresponds to the most recently detected object.
  private:  std::string object_frame_;

  /// \brief The name of the artifact origin frame.
  private: std::string artifact_origin_frame_;

  /// \brief Keeps track of recent positions of other robots to help avoid
  /// false positive artifact detections
  private: RobotPositionRadar robot_radar_;

  /// \brief Defines the max distance another robot can be from an artifact
  /// so that the artifact detection is considered a false positive
  private: float external_robot_tolerance_;
};

#endif
