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
#include <blob_detection/blob_detection.hh>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

BlobDetector::BlobDetector():
  imgSub_(this->nh_.subscribe("input/image_raw", 1, &BlobDetector::detectBlob, this)),
  BBPub_(this->nh_.advertise<darknet_ros_msgs::BoundingBoxes>("output/bounding_boxes", 1)),
  imPub_(this->nh_.advertise<sensor_msgs::Image>("output/debug_image", 1))
{
  ROS_INFO("BlobDetector Ready");
};

int BlobDetector::maxContour (const std::vector <std::vector<cv::Point>> & contours) {
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = j;
        }
    }
    if (maxAreaContourId == -1) return 0;
    return maxAreaContourId;
}

void BlobDetector::detect_color(cv::Scalar low, cv::Scalar high, const cv::Mat & image,
  cv::Mat & range, darknet_ros_msgs::BoundingBoxes & result,
  bool ero_dilate, const std::string & class_name, cv::Mat & debug_image)
{
  cv::Mat HSV_img;
  cv::cvtColor(image, HSV_img, cv::COLOR_BGR2HSV);
  cv::inRange(HSV_img, low, high, range);
  cv::blur(range, range, cv::Size(3,3));

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  if (ero_dilate) {

    // Following https://docs.opencv.org/2.4/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html, thats how erosion + dilation should be used
    // to isolate bodys from sparse points
    int dilatation = 2;
    int erosion = 2;
    cv::erode( range, range, cv::getStructuringElement(cv::MORPH_ELLIPSE,
      cv::Size(erosion * 2 + 1, erosion * 2 + 1),
      cv::Point(erosion, erosion)));

    cv::dilate( range, range, cv::getStructuringElement(cv::MORPH_ELLIPSE,
      cv::Size(dilatation * 2 + 1, dilatation * 2 + 1),
      cv::Point(dilatation, dilatation)));

    cv::dilate( range, range, cv::getStructuringElement(cv::MORPH_ELLIPSE,
      cv::Size(dilatation * 2 + 1, dilatation * 2 + 1),
      cv::Point(dilatation, dilatation)));

    cv::erode( range, range, cv::getStructuringElement(cv::MORPH_ELLIPSE,
      cv::Size(erosion * 2 + 1, erosion * 2 + 1),
      cv::Point(erosion, erosion)));
  }

  try {
    cv::findContours(range, contours, hierarchy, CV_RETR_EXTERNAL,
      CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    if (contours.size() > 0) {
      cv::Rect bounding_rect =
        cv::boundingRect(contours.at(maxContour(contours)));
      // check if the bounding rectangle is bigger than some decent pixel area
      if (bounding_rect.area() > 5000) {
        cv::rectangle( debug_image, bounding_rect.tl(),
          bounding_rect.br(), cv::Scalar(255,255,255), 2);
        cv::rectangle( range, bounding_rect.tl(),
          bounding_rect.br(), cv::Scalar(255), CV_FILLED);
        darknet_ros_msgs::BoundingBox box;
        box.xmin = bounding_rect.tl().x;
        box.ymin = bounding_rect.tl().y;
        box.xmax = bounding_rect.br().x;
        box.ymax = bounding_rect.br().y;
        box.Class = class_name;
        box.probability = 1;
        result.bounding_boxes.push_back(box);
      }
    }
  }
  catch (std::runtime_error & ex) {
    ROS_WARN_STREAM("Error searching for " << class_name << ": " << ex.what() << "/n");
  }
}


void BlobDetector::detectBlob(const sensor_msgs::Image::ConstPtr & img)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
    {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_WARN("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat detected_image, debug_image;
    cv_ptr->image.copyTo(detected_image);
    cv_ptr->image.copyTo(debug_image);

    cv::rectangle(detected_image, cv::Point(0,0), cv::Point(cv_ptr->image.cols, cv_ptr->image.rows / 10), cv::Scalar(0), CV_FILLED);

    darknet_ros_msgs::BoundingBoxes result;
    result.image_header = img->header;
    result.header.stamp = result.image_header.stamp;
    result.header.frame_id = "detection";

    cv::Mat range_red, range_yellow, range_blue, range_white;

    // Detect backpacks
    detect_color(cv::Scalar(0,163,41), cv::Scalar(3,255,120),
      detected_image, range_red, result, false, "backpack", debug_image);

    detected_image.setTo(cv::Scalar(0), range_red);

    // Detect people
    detect_color(cv::Scalar(33, 173, 90), cv::Scalar(34, 212, 136),
      detected_image, range_yellow, result, false, "person", debug_image);

    detected_image.setTo(cv::Scalar(0), range_yellow);

    // Detect ropes
    detect_color(cv::Scalar(71, 44, 37), cv::Scalar(130, 155, 142),
      detected_image, range_blue, result, false, "rope", debug_image);

    detected_image.setTo(cv::Scalar(0), range_blue);

    // Helmets are missing, cave has the same HSV range in some parts than helmets, so some other way is needed

    if (result.bounding_boxes.size() > 0) {
      BBPub_.publish(result);
      cv_ptr->image = debug_image;
      imPub_.publish(cv_ptr->toImageMsg());
    }
}
