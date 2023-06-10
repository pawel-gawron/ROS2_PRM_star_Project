/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include <random>
#include "nav2_util/node_utils.hpp"
#include <algorithm>

#include "nav2_straightline_planner/straight_line_planner.hpp"

namespace nav2_straightline_planner
{

void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void StraightLine::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}


//CUSTOM FUNCTIONS

float StraightLine::heuristic_cost(std::vector<float> point, std::vector<float> end){
    return sqrt(pow(end[0] - point[0], 2)+ pow(end[1] - point[1], 2));
}

std::vector<std::pair<double, double>> StraightLine::random_point(const std::pair<double, double>& start, const std::pair<double, double>& end) {
    std::vector<std::pair<double, double>> points;
    points.push_back(start);
    points.push_back(end);

    std::random_device rd;
    std::mt19937 gen(rd());
    double originX = costmap_->getOriginX();
    double originY = costmap_->getOriginY();

    int width = costmap_->getSizeInMetersX();
    int height = costmap_->getSizeInMetersY();
    RCLCPP_INFO(
    node_->get_logger(), "Dimensions: height=%d, width=%d",
    height, width);
    std::uniform_real_distribution<double> distrib_x(originX, originX + width);
    std::uniform_real_distribution<double> distrib_y(originY, originY + height);

    do {
        double x = distrib_x(gen);
        double y = distrib_y(gen);

        RCLCPP_INFO(
        node_->get_logger(), "Random: height=%f, width=%f",
        x, y);

        // VARIABLES UNUSED
        // int x_check = static_cast<int>(x / interpolation_resolution_);
        // int y_check = static_cast<int>(y / interpolation_resolution_);

        unsigned int mx,my;

        costmap_->worldToMap(x,y,mx,my);

        RCLCPP_INFO(
        node_->get_logger(), "world to map: height=%d, width=%d",
        costmap_->getCost(mx, my), my);

        if (costmap_->getCost(mx, my) < 200) {
            points.push_back(std::make_pair(x, y));
        }
    }
    while (points.size()<num_samples);

    return points;
}

bool StraightLine::isValid(const vertex& a, const vertex& b)
{
  return true;
    std::vector<double> x_div(100);
    std::vector<double> y_div(100);

    for (int i = 0; i < 100; ++i)
    {
        double t = static_cast<double>(i) / 99.0;
        x_div[i] = a.x + t * (b.x - a.x);
        y_div[i] = a.y + t * (b.y - a.y);
    }

    for (int i = 0; i < 100; ++i)
    {
        unsigned int mx,my;

        costmap_->worldToMap(x_div[i], y_div[i], mx, my);

        if (costmap_->getCost(mx, my) > 200) {
          return true;
        }
    }
    return true;
}

std::unordered_map<vertex,vertex,VertexHash> StraightLine::search(vertex start, vertex end)
{
  std::vector<std::pair<q, vertex>> queue;

  while !(queue.empty())
  {
    continue;
  }
  return std::unordered_map<vertex, vertex, VertexHash>();
}
bool compareBySecond(const std::pair<vertex, double> &a, const std::pair<vertex, double> &b)
{
    return a.second < b.second;
}

void StraightLine::computeNeighbours(vertex v, double radius, int K)
{
  node tmp;
  for (const auto& vertex : graph)
  {
    // std::vector<double> distances;

    if (v.x != vertex.first.x || v.y != vertex.first.y)
      {
                RCLCPP_INFO(
        node_->get_logger(), "Twoj stary");
        // if (isValid(v, vertex.first))
        // {
          double dist = sqrt(pow(v.x - vertex.first.x, 2)+ pow(v.y - vertex.first.y, 2));
          // if (dist <= radius)
          // {
            tmp.v.x = vertex.first.x;
            tmp.v.y = vertex.first.y;
            tmp.neighbours.push_back(std::make_pair(vertex.first, dist));
          // }
        // }
      }
  }

  std::sort(tmp.neighbours.begin(), tmp.neighbours.end(), nav2_straightline_planner::compareBySecond);

  // tmp.neighbours.resize(K);

  graph[v] = tmp;
}


// void StraightLine::addNeighbours(vertex v)
// {
//   node tmp;
//   tmp.v = v;
//   tmp.neighbours = StraightLine::computeNeighbours;
//   graph[v] = tmp;
// }


// const std::vector<std::tuple<std::pair<double, double>, double>>& StraightLine::StraightLine(const std::pair<double, double>& vertex)
// {
//   return adjacencyList[vertex];
// }

// std::vector<std::pair<double, double>> StraightLine::computeNeighbours(const std::pair<double, double>& vertex, double radius, int K)
// {

// }



////////////////////////////////////////////////////////////////////////////

//CUSTOM FUNCTIONS
////////////////////////////////////////////////////////////////////////////
nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  std::pair<double, double> startPose = std::make_pair(start.pose.position.x, start.pose.position.y);
  std::pair<double, double> endPose = std::make_pair(goal.pose.position.x, goal.pose.position.y);

  if (createPointsMap == true){
      random_points = StraightLine::random_point(startPose, endPose);
      createPointsMap = false;
  }

  // for (const auto& point : random_points) {
  //   RCLCPP_INFO(
  //       node_->get_logger(), "Random Point: x=%f, y=%f",
  //       point.first, point.second);
  // }

    // Używanie wygenerowanych punktów
  for (const auto& point : random_points) {
    // Dodawanie punktów do globalnej ścieżki
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    global_path.poses.push_back(pose);
  }

  // for (int i = 0; i < total_number_of_loop; ++i) {
  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.pose.position.x = start.pose.position.x + x_increment * i;
  //   pose.pose.position.y = start.pose.position.y + y_increment * i;
  //   pose.pose.position.z = 0.0;
  //   pose.pose.orientation.x = 0.0;
  //   pose.pose.orientation.y = 0.0;
  //   pose.pose.orientation.z = 0.0;
  //   pose.pose.orientation.w = 1.0;
  //   pose.header.stamp = node_->now();
  //   pose.header.frame_id = global_frame_;
  //   global_path.poses.push_back(pose);
  // }

  // geometry_msgs::msg::PoseStamped goal_pose = goal;
  // goal_pose.header.stamp = node_->now();
  // goal_pose.header.frame_id = global_frame_;
  // global_path.poses.push_back(goal_pose);

  //pobieramy kordynaty początku i końca
  unsigned int end_x,end_y;
  costmap_->worldToMap(goal.pose.position.x,goal.pose.position.y,end_x,end_y);
  unsigned int start_x,start_y;
  costmap_->worldToMap(start.pose.position.x,start.pose.position.y,start_x,start_y);

  // inicjalizacja pustego grafu
  std::vector<std::pair<double, double>> emptyVector;
  // computeNeighbours(vertex& v, double radius, int K);
  vertex startPosevertex;
  startPosevertex.x = 1.0;
  startPosevertex.y = 1.2;

  vertex startPosevertex3;
  startPosevertex3.x = 0.5;
  startPosevertex3.y = 0.3;

  vertex startPosevertex4;
  startPosevertex4.x = 1.0;
  startPosevertex4.y = 1.0;

  vertex startPosevertex5;
  startPosevertex5.x = 1.0;
  startPosevertex5.y = 0.8;

  vertex startPosevertex2;
  startPosevertex2.x = 1.1;
  startPosevertex2.y = 1.5;

  graph[startPosevertex] = node();
  // graph[startPosevertex].v = startPosevertex;

  graph[startPosevertex3] = node();
  // graph[startPosevertex3].v = startPosevertex3;

  graph[startPosevertex4] = node();
  // graph[startPosevertex4].v = startPosevertex4;

  graph[startPosevertex5] = node();
  // graph[startPosevertex5].v = startPosevertex5;
  computeNeighbours(startPosevertex, 1000.0, 5);
  // computeNeighbours(startPosevertex2, 1000.0, 5);

  // RCLCPP_INFO(
  // node_->get_logger(), "Twoja stara!!!");

  
  for (const auto& vertex : graph)
  {
    RCLCPP_INFO(
    node_->get_logger(), "Twoja stara %lf,  %lf,  %ld,  %ld", vertex.first.x, vertex.first.y, vertex.second.neighbours.size(), vertex.second.neighbours.size());
      // std::cout << vertex.first.x << "   " << vertex.first.y << "   " << vertex.second.neighbours[0].first.x << "   " << vertex.second.neighbours[0].first.y << std::endl;
  }


  return global_path;
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
#include "straight_line_planner.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
