// Copyright 2018 Toyota Research Institute.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "jaguar4x4_nav_msgs/srv/go_to_goal_pose.hpp" // pattern is all lower case name w/ underscores

class Jaguar4x4Nav final : public rclcpp::Node
{
public:
  Jaguar4x4Nav() : Node("jaguar4x4nav")
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",
                                                                     rmw_qos_profile_sensor_data);

    go_to_goal_pose_srv_ = this->create_service<jaguar4x4_nav_msgs::srv::GoToGoalPose>(
        "go_to_goal_pose",
        std::bind(&Jaguar4x4Nav::goToGoalPose, this,
                  std::placeholders::_1, std::placeholders::_2));

    // We use a separate callback group for the odometry callback so that a
    // thread can service this separately from the service callback.
    odom_cb_grp_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        std::bind(&Jaguar4x4Nav::odomCallback, this, std::placeholders::_1),
        rmw_qos_profile_sensor_data,
        odom_cb_grp_);
  }

  ~Jaguar4x4Nav()
  {
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::unique_lock<std::timed_mutex> odom_lk(odom_mutex_, std::defer_lock);
    bool got_lock = odom_lk.try_lock_for(std::chrono::milliseconds(100));
    if (!got_lock) {
      RCLCPP_INFO(get_logger(), "Could not acquire odom mutex before timeout");
      return;
    }
    odom_ = msg;
    odom_cv_.notify_one();
  }

  std::string goToGoalXY(double goal_x_m, double goal_y_m)
  {
    std::string error;
    double vel_const = 0.3;
    double h_const = 1.5;
    double v_max = 0.6; // don't go faster than this - it's scary
    double omega_max = 0.15; // don't go faster than this - it's scary

    double x_diff;
    double y_diff;
    double distance;
    double goal_theta_world_frame;
    double velocity;
    double omega;

    std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> last_pose_change_time = start_time;

    double last_x = 0.0;
    double last_y = 0.0;

    while (true) {
      std::string error_odom = waitForOdom();
      if (!error.empty()) {
        error += error_odom;
        break;
      }
      nav_msgs::msg::Odometry odom_copy;
      {
        std::unique_lock<std::timed_mutex> odom_lk(odom_mutex_, std::defer_lock);
        bool got_lock = odom_lk.try_lock_for(std::chrono::milliseconds(100));
        if (!got_lock) {
          RCLCPP_INFO(get_logger(), "Could not acquire odom mutex to copy odom");
          error += "Couldn't acquire odom mutex to copy odom!";
          break;
        }
        odom_copy.pose.pose.position.x = odom_->pose.pose.position.x;
        odom_copy.pose.pose.position.y = odom_->pose.pose.position.y;
        odom_copy.pose.pose.position.z = odom_->pose.pose.position.z;
        odom_copy.pose.pose.orientation.x = odom_->pose.pose.orientation.x;
        odom_copy.pose.pose.orientation.y = odom_->pose.pose.orientation.y;
        odom_copy.pose.pose.orientation.z = odom_->pose.pose.orientation.z;
        odom_copy.pose.pose.orientation.w = odom_->pose.pose.orientation.w;
      }

      if (odom_copy.pose.pose.position.x != last_x ||
          odom_copy.pose.pose.position.y != last_y) {
        last_pose_change_time = std::chrono::system_clock::now();
        last_x = odom_copy.pose.pose.position.x;
        last_y = odom_copy.pose.pose.position.y;
      }

      std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
      auto last_pose_change_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_pose_change_time);
      if (last_pose_change_diff_ms.count() > 5000) {
        // We didn't have any changes in pose in the last 5 seconds; assume we
        // are stuck or eStopped and fail the service.
        RCLCPP_INFO(get_logger(), "No movement in 5 seconds, giving up");
        error += "No movement in 5 seconds ";
        break;
      }

      x_diff = goal_x_m - odom_copy.pose.pose.position.x;
      y_diff = goal_y_m - odom_copy.pose.pose.position.y;
      goal_theta_world_frame = atan2(y_diff, x_diff);

      distance = sqrt(x_diff*x_diff + y_diff*y_diff);
      velocity = vel_const * distance;

      // TODO: we actually want to be within a bounding box of the goal
      if (distance < 0.05) {
        // we've gone the goal distance
        break;
      }

      if (fabs(velocity) > v_max) {
        if (velocity > 0.0) {
          velocity = v_max;
        } else {
          velocity = -v_max;
        }
      }

      double roll;
      double pitch;
      double yaw;
      tf2::Quaternion quat{odom_copy.pose.pose.orientation.x, odom_copy.pose.pose.orientation.y,
                           odom_copy.pose.pose.orientation.z, odom_copy.pose.pose.orientation.w};
      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      omega = h_const * (goal_theta_world_frame - yaw);

      if (fabs(omega) > omega_max) {
        if (omega > 0.0) {
          omega = omega_max;
        } else {
          omega = -omega_max;
        }
      }

      RCLCPP_INFO(get_logger(), "x_diff: %f, y_diff: %f, distance: %f, yaw: %f, omega: %f, velocity: %f", x_diff, y_diff, distance, yaw, omega, velocity);

      auto cmd_vel_msg = std::make_shared<geometry_msgs::msg::Twist>();
      cmd_vel_msg->linear.x = velocity;
      cmd_vel_msg->linear.y = 0.0;
      cmd_vel_msg->linear.z = 0.0;
      cmd_vel_msg->angular.x = 0.0;
      cmd_vel_msg->angular.y = 0.0;
      cmd_vel_msg->angular.z = omega;
      cmd_vel_pub_->publish(cmd_vel_msg);
    }
    return error;
  }

  std::string goToGoalTheta(double goal_theta_rad)
  {
    std::string error;

    RCLCPP_INFO(get_logger(), "in goToGoalTheta with goal theta = %f", goal_theta_rad);

    std::unique_lock<std::timed_mutex> lk(odom_mutex_, std::defer_lock);
    bool got_lock = lk.try_lock_for(std::chrono::milliseconds(100));
    if (!got_lock) {
      error = "Failed to acquire lock";
      return error;
    }

    // current heading... Quaternion odom_->pose.pose.orientation
    // goal heading...    request->goal_theta_rad
    geometry_msgs::msg::Quaternion last_orientation = odom_->pose.pose.orientation;
    std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> last_pose_change_time = start_time;
    while (true) {
      std::cv_status cv_status = odom_cv_.wait_for(lk,
                                                       std::chrono::milliseconds(kNoOdomTimeoutMS));
      if (cv_status == std::cv_status::timeout) {
        RCLCPP_INFO(get_logger(), "No odom update in %d ms, giving up", kNoOdomTimeoutMS);
        error += "No data before timeout ";
        break;
      }

      if (odom_->pose.pose.orientation != last_orientation) {
        last_pose_change_time = std::chrono::system_clock::now();
        last_orientation = odom_->pose.pose.orientation;
      }
      std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
      auto last_pose_change_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_pose_change_time);
      if (last_pose_change_diff_ms.count() > 5000) {
        // We didn't have any changes in pose in the last 5 seconds; assume we
        // are stuck or eStopped and fail the service.
        RCLCPP_INFO(get_logger(), "No movement in 5 seconds, giving up");
        error += "No movement in 5 seconds ";
        break;
      }

      // Check if we have reached our goal yet.
      tf2::Quaternion current_quat;
      tf2::fromMsg(odom_->pose.pose.orientation, current_quat);
      tf2::Quaternion goal_quat;
      goal_quat.setRPY(0.0, 0.0, goal_theta_rad);
      double angle_abs_diff = current_quat.angleShortestPath(goal_quat);
      RCLCPP_INFO(get_logger(), "angle diff: %f", angle_abs_diff);
      if (std::isnan(angle_abs_diff)) {
        // how did this happen?
        RCLCPP_INFO(get_logger(), "Got NaN for angle difference, giving up");
        error += "got NaN for angle difference ";
        break;
      }
      if (fabs(angle_abs_diff) < .05) {
        break;
      }

      double roll;
      double pitch;
      double yaw;
      tf2::Quaternion difference = goal_quat * current_quat.inverse();
      tf2::Matrix3x3(difference).getRPY(roll, pitch, yaw);

      auto cmd_vel_msg = std::make_shared<geometry_msgs::msg::Twist>();
      cmd_vel_msg->linear.x = 0.0;
      cmd_vel_msg->linear.y = 0.0;
      cmd_vel_msg->linear.z = 0.0;
      cmd_vel_msg->angular.x = 0.0;
      cmd_vel_msg->angular.y = 0.0;
      if (yaw >= 0.0) {
        cmd_vel_msg->angular.z = 0.2;
      } else {
        cmd_vel_msg->angular.z = -0.2;
      }
      cmd_vel_pub_->publish(cmd_vel_msg);
    }

    return error;
  }

  std::string goToGoalPoseImpl(double goal_x_m, double goal_y_m, double goal_theta_rad)
  {
    std::string error = goToGoalXY(goal_x_m, goal_y_m);

    if (error.empty()) {
      error = goToGoalTheta(goal_theta_rad);
    }

    // Send a stop message.
    auto cmd_vel_msg = std::make_shared<geometry_msgs::msg::Twist>();
    cmd_vel_msg->linear.x = 0.0;
    cmd_vel_msg->linear.y = 0.0;
    cmd_vel_msg->linear.z = 0.0;
    cmd_vel_msg->angular.x = 0.0;
    cmd_vel_msg->angular.y = 0.0;
    cmd_vel_msg->angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel_msg);

    return error;
  }

  void goToGoalPose(const std::shared_ptr<jaguar4x4_nav_msgs::srv::GoToGoalPose::Request> request,
                    std::shared_ptr<jaguar4x4_nav_msgs::srv::GoToGoalPose::Response> response)
  {
    if (go_to_goal_pose_service_running_) {
      response->status = -1;
      response->message = "already running";
      return;
    }
    go_to_goal_pose_service_running_ = true;
    std::string error = goToGoalPoseImpl(request->goal_x_m, request->goal_y_m, request->goal_theta_rad);
    go_to_goal_pose_service_running_ = false;

    if (!error.empty()) {
      response->status = -1;
      response->message = error;
    } else {
      response->status = 0;
      response->message = "MOOOOOO";
    }

    RCLCPP_INFO(get_logger(), "End of service");
  }

  std::string waitForOdom()
  {
    // Wait for new odom__
    std::unique_lock<std::timed_mutex> lk(odom_mutex_, std::defer_lock);
    bool got_lock = lk.try_lock_for(std::chrono::milliseconds(100));
    if (!got_lock) {
      return "Failed to acquire lock";
    }

    std::cv_status cv_status = odom_cv_.wait_for(lk,
                                                 std::chrono::milliseconds(500));
    if (cv_status == std::cv_status::timeout) {
      return "No odom acquired before timeout";
    }

    return "";
  }

  const uint32_t kNoOdomTimeoutMS = 500;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                     cmd_vel_pub_;
  rclcpp::Service<jaguar4x4_nav_msgs::srv::GoToGoalPose>::SharedPtr           go_to_goal_pose_srv_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                    odom_sub_;
  nav_msgs::msg::Odometry::SharedPtr                                          odom_;
  std::timed_mutex                                                            odom_mutex_;
  std::condition_variable_any                                                 odom_cv_;
  bool                                                                        go_to_goal_pose_service_running_;
  rclcpp::callback_group::CallbackGroup::SharedPtr                            odom_cb_grp_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto nav = std::make_shared<Jaguar4x4Nav>();
  executor.add_node(nav);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
