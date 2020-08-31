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

#ifndef BLOB_DETECTION_BLOB_DETECTION_HH_
#define BLOB_DETECTION_BLOB_DETECTION_HH_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

/// \class BlobDetection blob_detection.hh
/// \brief Class for detecting color's blobs and its bounding boxes
class BlobDetector
{
  public:
    /// \brief Constructor
    BlobDetector();

    /// \brief Detects colors in a sensor_msgs::Image msg.
    /// Publishes the bounding boxes of the color blobs
    /// found by the detector algorithm.
    /// \param[in] img The sensor_msgs::Image to apply the detector.
    void detectBlob(const sensor_msgs::Image::ConstPtr & img);


    /// \brief Finds the biggest contour of a vector.
    /// Returns the index of the biggest contour of a vector of contours
    /// \param[in] coutours The vector to search the biggest contour from.
    int maxContour(const std::vector <std::vector<cv::Point>> & contours);

    /// \brief Applies the filters and adds bounding boxes to the result msg.
    /// Uses a group of filters and OpenCV operations to find the bounding
    /// boxes surrounding the artifact that matches the color between the lower
    /// and the higher threshold.
    /// \param[in] low The lower color threshold in BGR format.
    /// \param[in] high The higher color threshold in BGR format.
    /// \param[in] image The cv::mat generated from the sensor_msgs::Image msg.
    /// \param[in] range The result of the filters, for usage outside.
    /// \param[in] result The BoundingBoxes msg where the boxes should be added.
    /// \param[in] previous_range The range parameter from the function
    /// used with another color, so one artifact does not make 2 bounding boxes.
    /// \param[in] ero_dilate Decides if erosion + dilatation is needed.
    /// \param[in] class_name The name of the class detected.
    /// \param[in] debug_image Image that is going to be published to debug.
    void detect_color(cv::Scalar low, cv::Scalar high, const cv::Mat & image,
      cv::Mat & range, darknet_ros_msgs::BoundingBoxes & result,
      bool ero_dilate, const std::string & class_name, cv::Mat & debug_image);

  private:
    /// \brief ROS nodehandle. Used for subscribing and publishing
    ros::NodeHandle nh_;

    /// \brief Subscriber for incoming sensor_msgs::Image message.
    ros::Subscriber imgSub_;

    /// \brief Publisher for darknet::bounding_boxes msg.
    /// Used to publish the bounding boxes detected by the algorithm.
    ros::Publisher BBPub_;

    /// \brief Publisher for debugging sensor_msgs::Image msg.
    /// Used to publish images with positives detected.
    ros::Publisher imPub_;
};
#endif
