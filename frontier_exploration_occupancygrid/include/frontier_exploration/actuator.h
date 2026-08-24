#ifndef FRONTIER_EXPLORATION__ACTUATOR_HPP_
#define FRONTIER_EXPLORATION__ACTUATOR_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "frontier_exploration/msg/point_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mbf_msgs/action/move_base.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"

const double PI = 3.1415926535897932385;

namespace Actuator
{

using MoveBaseAction = mbf_msgs::action::MoveBase;
using MoveBaseClient = rclcpp_action::Client<MoveBaseAction>;
using MoveBaseGoalHandle = rclcpp_action::ClientGoalHandle<MoveBaseAction>;

struct Header_param
{
  std::string frame_id = "map";
  rclcpp::Time stamp = rclcpp::Clock().now();
};

struct RobotPose
{
  geometry_msgs::msg::Point Position;
  double Yaw;
};

// Actuator: select a goal from frontiers and drive the robot via the
// move_base (move_base_flex) action server.
class Actuator
{
private:
  rclcpp::Node::SharedPtr node_;

  std::string CmdTopic;
  std::string RobotBase;
  float GoalTolerance;     // abandon next goal close to an explored point
  float ObstacleTolerance; // do not rotate when close to an obstacle
  float RotateSpeed;
  long iteration;

  Header_param header;
  std::vector<geometry_msgs::msg::Point> centroids;
  geometry_msgs::msg::Point Home;  // the mapping origin
  visualization_msgs::msg::Marker GoalMarker;
  visualization_msgs::msg::Marker HomeMarker;
  nav_msgs::msg::OccupancyGrid raw_map;

  geometry_msgs::msg::Twist RotSpeed;
  RobotPose robotPose;  // robot pose through TF

  mbf_msgs::action::MoveBase::Goal MoveGoal;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_vis;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr home_vis;
  rclcpp::Subscription<frontier_exploration::msg::PointArray>::SharedPtr centroidsSub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr RawMapSub_;

  MoveBaseClient::SharedPtr ac_;
  MoveBaseGoalHandle::SharedPtr goal_handle_;

public:
  geometry_msgs::msg::Point Goal;
  std::vector<geometry_msgs::msg::Point> GoalClose;  // traveled / unavailable points
  int GoHomeFlag;

  Actuator(const rclcpp::Node::SharedPtr & node);
  ~Actuator();

  geometry_msgs::msg::Point SelectGoal(std::vector<geometry_msgs::msg::Point> & centroids);
  void ObtainPose();
  void MoveToGoal();
  void CancelGoal();
  void CancelAllGoals();
  void Rotation(float angle);
  void ReturnHome();
  void Visualization();
  void VisInit();
  void ActuatorInit();
  void AddToClose(geometry_msgs::msg::Point & goal);
  void centroidCallback(frontier_exploration::msg::PointArray::SharedPtr msg);
  void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr RawMap);

  rclcpp_action::GoalStatus GetGoalStatus() const;
  int CheckCollision(const nav_msgs::msg::OccupancyGrid & map,
                     geometry_msgs::msg::Point & start, geometry_msgs::msg::Point & end);
};

}  // namespace Actuator

#endif  // FRONTIER_EXPLORATION__ACTUATOR_HPP_
