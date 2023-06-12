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
  // std::unordered_map<vertex, node, VertexHash> StraightLine::getGraph()
  // {
  //   return graph;
  // }
  // node StraightLine::getGraphNode(vertex v)
  // {
  //   return graph[v];
  // }
  // void StraightLine::setGraph(std::unordered_map<vertex, node, VertexHash> newGraph)
  // {
  //   graph =  newGraph;
  // }
  // void StraightLine::setGraphNode(vertex v, node n)
  // {
  //   graph[v] = n;
  // }

float StraightLine::heuristic_cost(vertex point, vertex end){
    return sqrt(pow(end.x - point.x, 2)+ pow(end.y - point.y, 2));
}

std::vector<vertex> StraightLine::random_point(const std::pair<double, double>& start, const std::pair<double, double>& end, double originX, double originY) {
    std::vector<vertex> points;
    vertex tmp_start;
    tmp_start.x = start.first;
    tmp_start.y = start.second;

    vertex tmp_end;
    tmp_end.x = end.first;
    tmp_end.y = end.second;

    points.push_back(tmp_start);
    points.push_back(tmp_end);

    std::random_device rd;
    std::mt19937 gen(rd());
    // double originX = costmap_->getOriginX();
    // double originY = costmap_->getOriginY();

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
        mx, my);

        if (costmap_->getCost(mx, my) < 150) {
            vertex tmp;
            tmp.x = x;
            tmp.y = y;
            points.push_back(tmp);
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
  // TODO: check if heuristtic is needed
  std::vector<q> queue_vec;
  q start_q;
  start_q.v = start;
  start_q.distance = heuristic_cost(start, end);
  queue_vec.push_back(start_q);
  int i = 0;

  std::set<vertex> visited;
  start_dist[start] = 0.0;

  vertex current;
  while (!queue_vec.empty())
  {
    // i++;

    // RCLCPP_INFO(
    // node_->get_logger(), "Search: %d", i);

    current = queue_vec.front().v;
    queue_vec.erase(queue_vec.begin());

    // RCLCPP_INFO(
    // node_->get_logger(), "currentX: %f, currentY: %f", current.x, current.y);
    double heuristic = queue_vec.front().distance;

    if (current.x == end.x && current.y == end.y){
      // FINISHED
      // StraightLine::constructPath();
      RCLCPP_INFO(
      node_->get_logger(), "PATH FOUND!");
      return parent_unordered_map;
    }

    if (visited.find(current) != visited.end()) continue;

    visited.insert(current);

    std::vector<std::pair<vertex, double>> neighbours = graph[current].neighbours; 

    // RCLCPP_INFO(
    // node_->get_logger(), "Neighbours size: %ld", neighbours.size());

    vertex prev_neighbour;
    double prev_distance = 1.0/0.0; // approximation of infinity value

    for (const auto& pair : neighbours) 
    {
      vertex neighbour = pair.first;
      double distance = pair.second;

        //       RCLCPP_INFO(
        // node_->get_logger(), "####################################################");


        //       RCLCPP_INFO(
        // node_->get_logger(), "NeighbourX: %f,  NeighbourY: %f", neighbour.x, neighbour.y);

      double new_distance = start_dist[current] + distance;
      if (start_dist.find(neighbour) != start_dist.end() || \
      ((prev_neighbour.x == 0.0 && prev_neighbour.y == 0.0) \
      && new_distance + heuristic_cost(neighbour, end) < prev_distance) + heuristic_cost(prev_neighbour, end))
        {
        // RCLCPP_INFO(
        // node_->get_logger(), "####################################################");
          // parent_unordered_map[neighbour] = current;
          parent_unordered_map.insert(std::make_pair(neighbour, current));
          start_dist[neighbour] = new_distance;
          prev_neighbour = neighbour;
          prev_distance = new_distance;
        }

        // RCLCPP_INFO(
        // node_->get_logger(), "Before push back: %ld", queue_vec.size());

      q tmp = {new_distance + heuristic_cost(neighbour, end), neighbour};
      queue_vec.push_back(tmp);

        // RCLCPP_INFO(
        // node_->get_logger(), "After push back: %ld", queue_vec.size());

    if (graph.find(current) != graph.end()) {
    // std::unordered_map<vertex, node, VertexHash> graph_copy = getGraph();
    // graph_copy.erase(current);
    graph.erase(current);
    for (auto& node_neighbors : graph) {
        auto& neighbors = node_neighbors.second;
        neighbours.erase(std::remove_if(neighbours.begin(), neighbours.end(), 
            [current](const std::pair<vertex, double>& neighbor) {
                return neighbor.first == current;
            }), neighbours.end());
      }
    }
    
      // self.publish_search() ???
    }
  }
  RCLCPP_INFO(
  node_->get_logger(), "PATH NOT FOUND");
  return {};
}

std::vector<vertex> StraightLine::constructPath(std::unordered_map<vertex, vertex, VertexHash> connections, vertex start, vertex end)
{
  // std::vector<vertex> path;
  // path.push_back(end);
  // vertex current = end;

  // // for (const auto& point : parent_unordered_map) {
  // //   RCLCPP_INFO(
  // //   node_->get_logger(), "NodeX: %f, NodeY: %f", point.first.x, point.first.y);
  // // }

  // while (current != end)
  // {
  //   path.push_back(connections[current]);
  //   current = connections[current];
  // }
  // path.push_back(start);
  
  // return path;

    std::vector<vertex> path;
  path.push_back(end);
  vertex current = end;

  while (current != start)
  {
    current = connections[current];
    path.push_back(current);
  }

  std::reverse(path.begin(), path.end()); // Odwrócenie wektora, aby mieć ścieżkę od startu do końca.

  return path;
}


bool compareBySecond(const std::pair<vertex, double> &a, const std::pair<vertex, double> &b)
{
    return a.second < b.second;
}

void StraightLine::computeNeighbours(vertex v, double radius, int K)
{
  node tmp;
  int temp = 0;

  for (const auto& vertex_pair : graph)
  {

    const vertex& vertex_temp = vertex_pair.first;

    if (v != vertex_temp)
    {
      if (isValid(v, vertex_temp))
      {
        double dist = sqrt(pow(v.x - vertex_temp.x, 2) + pow(v.y - vertex_temp.y, 2));
        if (dist <= radius)
        {
              // temp +=1;
  //     RCLCPP_INFO(
  // node_->get_logger(), "COUNTER: %d", temp);
          tmp.v.x = vertex_temp.x;
          tmp.v.y = vertex_temp.y;
          tmp.neighbours.push_back(std::make_pair(vertex_temp, dist));
        }
      }
    }
  }

  // Assuming you want to update the node in the graph
  // tmp.neighbours.resize(K);
  graph[v] = tmp;
  std::size_t mapSize = graph[v].neighbours.size();
  RCLCPP_INFO(
  node_->get_logger(), "GRAPH SIZE: %zu", mapSize);
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

  //pobieramy kordynaty początku i końca
  unsigned int end_x,end_y;
  costmap_->worldToMap(goal.pose.position.x,goal.pose.position.y,end_x,end_y);
  unsigned int start_x,start_y;
  costmap_->worldToMap(start.pose.position.x,start.pose.position.y,start_x,start_y);


  double originX = costmap_->getOriginX();
  double originY = costmap_->getOriginY();

  unsigned int origin_x,origin_y;
  costmap_->worldToMap(originX,originY,origin_x,origin_y);

  startPointX = start.pose.position.x + origin_x;
  startPointY = start.pose.position.y + origin_y;
  endPointX = goal.pose.position.x + origin_x;
  endPointY = goal.pose.position.y + origin_y;


  std::pair<double, double> startPose = std::make_pair(startPointX, startPointY);
  std::pair<double, double> endPose = std::make_pair(endPointX, endPointY);

  vertex tmp_start;
  tmp_start.x = startPointX;
  tmp_start.y = startPointY;

  vertex tmp_end;
  tmp_end.x = endPointX;
  tmp_end.y = endPointY;

      // RCLCPP_INFO(
      // node_->get_logger(), "Start: %s Finish:",
      // global_frame_.c_str());

  if (createPointsMap == true){
      random_points = StraightLine::random_point(startPose, endPose, originX, originY);
      createPointsMap = false;

          // Używanie wygenerowanych punktów
  for (const auto& point : random_points) {
    // Dodawanie punktów do globalnej ścieżki
    // geometry_msgs::msg::PoseStamped pose;
    // pose.header.frame_id = global_frame_;
    // pose.pose.position.x = point.x;
    // pose.pose.position.y = point.y;
    // global_path.poses.push_back(pose);

    // add every point to graph as empty

    graph[point] = node();
  }

  for (const auto& v : random_points)
  {
    computeNeighbours(v, 0.5, 5);
  }

    //   for (const auto& vertex : graph)
    // {
    //   RCLCPP_INFO(
    //   node_->get_logger(), "Test %lf,  %lf,  %zu,  %zu", vertex.first.x, vertex.first.y, vertex.second.neighbours.size(), vertex.second.neighbours.size());
    //     // std::cout << vertex.first.x << "   " << vertex.first.y << "   " << vertex.second.neighbours[0].first.x << "   " << vertex.second.neighbours[0].first.y << std::endl;
    // }

  }

  // for (const auto& point : random_points) {
  //   RCLCPP_INFO(
  //       node_->get_logger(), "Random Point: x=%f, y=%f",
  //       point.first, point.second);
  // }

  std::unordered_map<vertex, vertex, VertexHash> map;

  map = search(tmp_start, tmp_end);


  std::vector<vertex> path = constructPath(map, tmp_start, tmp_end);

  RCLCPP_ERROR(
  node_->get_logger(), "StartX: %f startY: %f goalX: %f goalY: %f originX: %f  originY: %f",
  startPointX, startPointY, endPointX, endPointY, originX, originY);

  for (const auto& point : path) {
      // Dodawanie punktów do globalnej ścieżki
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      global_path.poses.push_back(pose);

      RCLCPP_INFO(
      node_->get_logger(), "Points path: %f,  %f", point.x, point.y);
  }
  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = node_->now();
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);


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
  // unsigned int end_x,end_y;
  // costmap_->worldToMap(goal.pose.position.x,goal.pose.position.y,end_x,end_y);
  // unsigned int start_x,start_y;
  // costmap_->worldToMap(start.pose.position.x,start.pose.position.y,start_x,start_y);

  return global_path;
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
