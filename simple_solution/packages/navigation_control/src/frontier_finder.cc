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

#include <frontier_finder/frontier_finder.hh>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <base_local_planner/line_iterator.h>
#include <nav_msgs/GetPlan.h>

using namespace frontier_finder;

FrontierFinder::FrontierFinder(
  const float frontier_dist,
  const float min_frontier_size,
  const float max_frontier_size,
  const float plan_tolerance
  ) :
  tf_listener_(this->tf_buffer_),
  costmap_client_(&this->tf_buffer_),
  world_model_(*this->costmap_client_.getCostmap()),
  min_frontier_size_(
    this->costmap_client_.getCostmap()->cellDistance(min_frontier_size)
  ),
  max_frontier_size_(
    this->costmap_client_.getCostmap()->cellDistance(max_frontier_size)
  ),
  points_in_circle_(
    getCirclePointsFromOctant(
      getOctantPoints(
        this->costmap_client_.getCostmap()->cellDistance(frontier_dist)
      )
    )
  ),
  plan_tolerance_(plan_tolerance),
  frontier_dist_(frontier_dist)
{
  ros::NodeHandle nh;
  std::string nodeName = ros::this_node::getName();

  nh.param<bool>(nodeName + "/debug", this->debug_, false);

  if(this->debug_) {
    this->frontier_pub_ = nh.advertise<visualization_msgs::MarkerArray>("frontiers", 1);
    this->frontier_candidates_pub_ = nh.advertise<visualization_msgs::MarkerArray>("candidate_frontiers", 1);
    this->circle_pub_ = nh.advertise<visualization_msgs::MarkerArray>("circle", 1);
  }
}

std::vector<geometry_msgs::Point> FrontierFinder::findFrontiers(const std::string & frame)
{
  point_t center = getRobotPoseInMap();
  std::vector<point_t> frontiers;
  std::vector<geometry_msgs::Point> frontiers_points;
  int first_occupied = getFirstOccupiedPointIndex(center);

  std::vector<point_t> debug_circle_free;
  std::vector<point_t> debug_circle_occupied;

  std::vector<point_t> curr_frontier;
  point_t curr_point;
  for(int i = 0; i < this->points_in_circle_.size(); i++) {
    curr_point = getCirclePointWithCenter(
      (i + first_occupied) % this->points_in_circle_.size() ,
      center
    );

    int length = curr_frontier.size();
    if(isObstacleFree(curr_point) && length < this->max_frontier_size_)
    {
      curr_frontier.push_back(curr_point);
      debug_circle_free.push_back(curr_point);
    }
    else
    {
      debug_circle_occupied.push_back(curr_point);
      if( length > 0)
      {
        if(length  > this->min_frontier_size_)
        {
          frontiers.push_back(
            curr_frontier.at(curr_frontier.size() / 2)
          );
        }
      curr_frontier.clear();
      }
    }
  }
  if(curr_frontier.size() > this->min_frontier_size_)
  {
    frontiers.push_back(
      curr_frontier.at(curr_frontier.size() / 2)
    );
  }

  std::vector<point_t> accesible_frontiers = getAccesibleFrontiers(frontiers);
  ROS_DEBUG("Accesible : %ld", accesible_frontiers.size() );

  frontiers_points = transformFrontiers(accesible_frontiers, frame);

  if(this->debug_) {
    deletePoints("frontiers", frame, this->frontier_pub_);
    publishPoints(frontiers_points, "frontiers", frame, this->frontier_pub_, 0.5, 0.0, 0.0, 1.0);
    std::vector<geometry_msgs::Point> candidate_frontiers_points = transformFrontiers(frontiers, frame);
    deletePoints("candidates", frame, this->frontier_candidates_pub_);
    publishPoints(candidate_frontiers_points,  "candidates", frame, this->frontier_candidates_pub_,0.5, 1.0, 0.0, 0.0);
    std::vector<geometry_msgs::Point> free_circle_points = transformFrontiers(debug_circle_free, frame);
    std::vector<geometry_msgs::Point> occupied_circle_points = transformFrontiers(debug_circle_occupied, frame);
    deletePoints("circle", frame, this->circle_pub_);
    publishPoints(free_circle_points,  "circle", frame, this->circle_pub_,0.1, 0.0, 1.0, 0.0);
    publishPoints(occupied_circle_points,  "circle", frame, this->circle_pub_,0.1, 1.0, 0.0, 0.0);
  }

  return frontiers_points;
}

std::vector<point_t> FrontierFinder::getOctantPoints(const int & radius) {

  int x = radius, y = 0, p = 1 - radius;
  std::vector<point_t> octant;

  octant.push_back(point_t(x,y));
  while (x > y)
  {
    y++;
    if (p <= 0)
    {
      p = p + 2 * y + 1;
    }
    else
    {
      x--;
      p = p + 2*y - 2*x + 1;
    }
    if (x < y)
    {
        break;
    }
    octant.push_back(point_t(x,y));
  }

  return octant;
}

std::vector<point_t> FrontierFinder::getCirclePointsFromOctant(
                                      const std::vector<point_t>& octant) {
  std::vector<point_t> circle;
  int first, second;
  point_t current;
  for(int i = 1 ; i <= 8; ++i) {
     for(int j = ( i % 2 ) ? 0 :octant.size() -1 ;
         ( i % 2) ? j < octant.size() : j >= 0;
         ( i % 2) ? j++ : j-- )
      {
          if( ((i + 2) % 4) > 1  )
          {
            first = octant.at(j).first;
            second = octant.at(j).second;
          }
          else
          {
            first = octant.at(j).second;
            second = octant.at(j).first;
          }
          current.first = first * (( ((i + 5) % 8) > 3 ) ? 1 : -1);
          current.second = second * (( ((i + 3) % 8) > 3 ) ? 1 : -1);

          if(circle.size() == 0 || circle.back() != current)
          {
            circle.push_back(current);
          }

     }
  }
  return circle;
}

point_t FrontierFinder::getRobotPoseInMap() {
  geometry_msgs::PoseStamped curr_pose = this->costmap_client_.getRobotPose();

  point_t ret;
  this->costmap_client_.getCostmap()->worldToMapEnforceBounds(
                                           curr_pose.pose.position.x,
                                           curr_pose.pose.position.y,
                                           ret.first, ret.second);

  return ret;
}

point_t FrontierFinder::getCirclePointWithCenter(const int & circle_index,
                                                      const point_t & center ) {

  point_t ret_point = this->points_in_circle_.at(circle_index);

  ret_point.first = ret_point.first + center.first;
  ret_point.second = ret_point.second + center.second;

  return ret_point;
}

bool FrontierFinder::isObstacleFree(const point_t& point)
{
  double cost = this->world_model_.pointCost(point.first, point.second);
  //return cost <= 0 && cost != -1.0;
    return cost <= 128 && cost != -1.0;
}

int FrontierFinder::getFirstOccupiedPointIndex(const point_t & center )
{
    int i = 0;
    for(;
      i < this->points_in_circle_.size() && isObstacleFree(
                                            getCirclePointWithCenter(i, center)
                                          );
      i++) {}
    return i;
}

std::vector<geometry_msgs::Point> FrontierFinder::transformFrontiers(
  const std::vector<point_t>& frontiers, std::string frame_id)
{
  std::vector<geometry_msgs::Point> transformed_frontiers;

  point_t robot_pose = getRobotPoseInMap();

  geometry_msgs::PointStamped transformed_frontier;
  geometry_msgs::PointStamped curr_frontier;
  curr_frontier.header.frame_id = this->costmap_client_.getGlobalFrameID();
  geometry_msgs::Point curr_point;
  double cost;
  for(int i=0; i < frontiers.size(); i++){

    this->costmap_client_.getCostmap()->mapToWorld(
        frontiers.at(i).first, frontiers.at(i).second,
        curr_frontier.point.x, curr_frontier.point.y
    );
    try
    {
      this->tf_buffer_.transform<geometry_msgs::PointStamped>(
         curr_frontier,
         transformed_frontier,
         frame_id
         );
    }
    catch(const tf2::TransformException & ex)
    {
     ROS_WARN("tf2::TransformException in FrontierFinder::findFrontiers(): %s",
                                                                      ex.what());
    }
    curr_point.x = transformed_frontier.point.x;
    curr_point.y = transformed_frontier.point.y;
    transformed_frontiers.push_back(curr_point);
  }

  return transformed_frontiers;
}

std::vector<point_t> FrontierFinder::getAccesibleFrontiers(
                                        const std::vector<point_t> & frontiers)
{

  std::vector<point_t> accesible_frontiers;

  for(int i=0; i < frontiers.size(); i++){
    if(isFrontierAccesible(frontiers.at(i))){
      accesible_frontiers.push_back(frontiers.at(i));
    }
  }

  return accesible_frontiers;

}

bool FrontierFinder::isFrontierAccesible(const point_t & point){

  std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::PoseStamped frontier;

  //Create a frontier point in the world
  frontier.header.frame_id = this->costmap_client_.getGlobalFrameID();
  this->costmap_client_.getCostmap()->mapToWorld(
        point.first, point.second,
        frontier.pose.position.x, frontier.pose.position.y
  );

  //Get the robot pose in the world
  geometry_msgs::PoseStamped current_location = this->costmap_client_.getRobotPose();

  // Get a plan from the robot pose to the frontier
  if( ! this->getPlan( frontier, current_location, this->plan_tolerance_, plan) )
  {
      ROS_DEBUG("NO PLAN");
      return false;
  }

  //Check that each point of the plan is inside the "circle"
  for (const auto & frontier_point : plan) {
     if(round(sqrt(
           pow(current_location.pose.position.x - frontier_point.pose.position.x, 2.0)
         + pow(current_location.pose.position.y - frontier_point.pose.position.y, 2.0)
       )) > this->frontier_dist_ )
       {
         ROS_DEBUG("OUT OF CIRCLE");
          return false;
       }
  }

  return true;
}

bool FrontierFinder::getPlan( const geometry_msgs::PoseStamped &start,
                              const geometry_msgs::PoseStamped &goal,
                              const float & tolerance,
                              std::vector<geometry_msgs::PoseStamped> &plan)
{

  nav_msgs::GetPlan srv;
  srv.request.start = start;
  srv.request.goal = goal;
  srv.request.tolerance = tolerance;

  ROS_DEBUG("\nABOUT TO GET PLAN");


  ROS_DEBUG("start: (%f,%f) - goal: (%f,%f) - tolerance: %f", srv.request.start.pose.position.x,
srv.request.start.pose.position.y, srv.request.goal.pose.position.x, srv.request.goal.pose.position.y,
srv.request.tolerance );



  if (ros::service::call("move_base/make_plan", srv)) //TODO unharcode this
  {

    if( srv.response.plan.poses.size() == 0)
    {
      ROS_DEBUG("NO PLAN FOUND");
      return false;
    }
    ROS_DEBUG("PLAN FOUND!");
    plan = srv.response.plan.poses;
    return true;
  }

  ROS_DEBUG("MOVE BASE BROKE Unable to get plan for frontier");

  return false;
}

void FrontierFinder::publishPoints(std::vector<geometry_msgs::Point> frontiers,
  const std::string &ns, const std::string & frame, const ros::Publisher & publisher,
  double scale, float r, float g, float b)
{

  visualization_msgs::MarkerArray msg;
  for (auto i = 0; i < frontiers.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.ns = ns;
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position = frontiers[i];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    msg.markers.push_back(marker);
  }

  // publish the newly found markers (frontiers)
  publisher.publish(msg);
}

void FrontierFinder::deletePoints(const std::string &ns, const std::string & frame,
                                  const ros::Publisher & publisher)
{

  visualization_msgs::MarkerArray delete_msg;
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::DELETEALL;
  delete_msg.markers.push_back(marker);
  publisher.publish(delete_msg);

}
