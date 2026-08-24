#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "frontier_exploration/actuator.h"
#include "frontier_exploration/frontier_detector.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("frontier_planner");

  time_t startTime, currTime, Duration;
  time_t Limit = 180;  // max time limitation for single goal navigation
  int changeFlag = 0;  // give up the current goal immediately if too close to an obstacle

  std::cout << "------------Exploration Starting------------" << std::endl;
  // create the frontier detector and actuator
  FrontierDetector::FrontierDetector frontier_detector(node);
  Actuator::Actuator actuator(node);
  // rotate 360 degrees to initialize the environment
  actuator.Rotation(360.0);

  // -------------------- Main Loop -------------------- //
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    // get all centroids
    frontier_detector.ComputeCentroids(frontier_detector.inflated_map, frontier_detector.frontier);
    // select a goal according to the cost value
    actuator.SelectGoal(frontier_detector.centroids);
    std::cout << "Found frontier cells: " << frontier_detector.frontier.size() << std::endl
              << "Found frontier: " << frontier_detector.centroids.size() << std::endl;
    // navigate to the goal
    actuator.MoveToGoal();
    time(&startTime);
    Duration = 0;
    // if the goal is abandoned, aborted, cancelled, or over time, select the next goal
    while (rclcpp::ok() &&
           actuator.GetGoalStatus().status != rclcpp_action::GoalStatus::STATUS_SUCCEEDED &&
           actuator.GetGoalStatus().status != rclcpp_action::GoalStatus::STATUS_ABORTED &&
           actuator.GetGoalStatus().status != rclcpp_action::GoalStatus::STATUS_CANCELED &&
           bool(Duration < Limit)) {
      rclcpp::spin_some(node);
      if (frontier_detector.GridValue(frontier_detector.inflated_map, actuator.Goal) >= 65) {
        changeFlag = 1;
        actuator.CancelAllGoals();
        std::cout << "Goal's close to obstacle. Changed Goal!!" << std::endl;
        break;
      }
      time(&currTime);
      Duration = currTime - startTime;  // avoid spending too much time on one goal
    }
    if (Duration >= Limit) { std::cout << "Overtime. Changed Goal!!" << std::endl; }
    actuator.AddToClose(actuator.Goal);  // add to closeList to avoid revisiting the explored goal
    if (changeFlag != 1 && Duration < Limit) {
      std::cout << "Reached the goal!" << std::endl;
      actuator.Rotation(0.0);
    } else {
      changeFlag = 0;
    }

    // Only treat "no frontiers" as exploration finished once a map has actually been
    // received; before that, keep looping and wait for the map (the ROS1 original hid
    // this via its initial 360-deg rotation, which the empty-map guard skips).
    if (!frontier_detector.inflated_map.data.empty() &&
        (frontier_detector.frontier.size() == 0 || actuator.GoHomeFlag == 1)) {  // for homing
      actuator.ReturnHome();
      while (rclcpp::ok() &&
             actuator.GetGoalStatus().status != rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
        std::cout << "Exploration finished! Returning home.." << std::endl;
        rclcpp::sleep_for(std::chrono::seconds(5));
        rclcpp::spin_some(node);
      }
      rclcpp::shutdown();
    }
  }
  rclcpp::shutdown();
  return 0;
}
