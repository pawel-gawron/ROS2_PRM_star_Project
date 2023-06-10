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

#ifndef NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <unordered_map>
#include <tuple>

namespace nav2_straightline_planner
{


struct vertex
{
  double x;
  double y;
  
  bool operator==(const vertex& other) const
  {
    return x == other.x && y == other.y;
  }
  bool operator!=(const vertex& other) const
  {
    return x != other.x || y != other.y;
  }

// probably not needed
  // vertex& operator=(const vertex& other)
  // {
  //   if (this == &other) {
  //         return *this;  // Self-assignment, no need to perform any operation
  //     }
      
  //     x = other.x;
  //     y = other.y;
      
  //     return *this;
  // }

      bool operator<(const vertex& other) const
    {
        // Define the comparison logic based on your requirements
        if (x < other.x)
            return true;
        else if (x > other.x)
            return false;
        else
            return y < other.y;
    }
};

struct node
{
  vertex v;
  std::vector<std::pair<vertex, double>> neighbours;
};

struct q
{
  double distance;
  vertex v;
};
// Custom hash function for vertex
struct VertexHash
{
  std::size_t operator()(const vertex& v) const
  {
    std::size_t h1 = std::hash<double>{}(v.x);
    std::size_t h2 = std::hash<double>{}(v.y);
    return h1 ^ (h2 << 1); // Combining the hash values
  }
};



class StraightLine : public nav2_core::GlobalPlanner
{
public:
  StraightLine() = default;
  ~StraightLine() = default;

  // plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

////////////////////////////////////////////////////////////////////////////
//CUSTOM FUNCTIONS
  std::unordered_map<vertex, node, VertexHash> graph;
  void computeNeighbours(vertex v, double radius, int K);
  float heuristic_cost(vertex point, vertex end);
  std::vector<std::pair<double, double>> random_point(const std::pair<double, double>& start, const std::pair<double, double>& end);

  bool isValid(const vertex& a, const vertex& b);
  
  std::unordered_map<vertex, vertex, VertexHash> search(vertex start, vertex end);
  std::unordered_map<vertex, double, VertexHash> start_dist;

  void constructPath(std::unordered_map<vertex, vertex, VertexHash> connections);

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
  std::unordered_map<vertex, vertex, VertexHash> parent;
private:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  double interpolation_resolution_;
  int num_samples = 200;

  bool createPointsMap = true;
  std::vector<std::pair<double, double>> random_points;
};




// CUSTOM CLASS
// Graph representation using adjacency list
// class Graph
// {
// public:
//     // map
//     // keys : vertex
//     // values : vector of neighbours 
//     // each neighbour is a tuple: <vertex>, distance    
//     std::unordered_map<vertex, node, VertexHash> graph;

//     void computeNeighbours(vertex v, double radius, int K);

//     // Add neighbours to vertex
//     // void addNeighbours(vertex v);

//     // void addNeighbours(std::pair<double, double> vertex, std::tuple<std::pair<double, double>>);

//     // Get the neighbors of a vertex
//     // const std::vector<std::tuple<std::pair<double, double>, double>>& getNeighbors(const std::pair<double, double>&);

//     // 
    

//     // std::tuple<std::pair<double, double>, double> computeNeighbours(<std::pair<double, double> vertex);



// };

}  // namespace nav2_straightline_planner


#endif  // NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
