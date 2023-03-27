#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <geometry_msgs/msg/pose_stamped.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>
#include <condition_variable>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <nav_msgs/msg/path.hpp>

#include "complete_coverage_planner/costmap_client.hpp"

using namespace std::placeholders;
#define NAV_POSE_ACTION "navigate_to_pose"
#define NAV_TO_POSES_ACTION "navigate_through_poses"
namespace complete_coverage_planner
{
/**
 * @class CompleteCoverage
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class CompleteCoverage : public rclcpp::Node
{
  //using NavigateThroughPose = nav2_msgs::action::NavigateThroughPoses;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using ActionT = nav2_msgs::action::NavigateToPose;
public:
  CompleteCoverage();
  ~CompleteCoverage();

void stop();

  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using Feedback = typename ActionT::Feedback;

private:
  /**
   * @brief  Execute Plane
   */
  void do_plan();
    /**
   * @brief  Make a complete coverage plan
   */
  void compute_plan();
  /**
   * @brief  Send a goal to the nav system
   */  
  void send_goal();

  // /**
  //  * @brief  Publish a frontiers as markers
  //  */

  bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  // void
  // goal_response_callback(std::shared_future<NavigationGoalHandle::SharedPtr>
  // future);
  void reachedGoal(const NavigationGoalHandle::WrappedResult& result);
  //used to pause navigation
  void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg);
  //used to start and stop navigation
  void start(const std_msgs::msg::Bool::SharedPtr msg);
  void feedbackCallback(std::shared_ptr<const nav2_msgs::action::NavigateToPose_Feedback_<std::allocator<void>>> );

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_publisher_;
  rclcpp::Logger logger_ = rclcpp::get_logger("CompleteCoverageNode");
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      move_base_client_;
  rclcpp::TimerBase::SharedPtr path_timer_;
  // rclcpp::TimerBase::SharedPtr oneshot_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_subscription_;
  geometry_msgs::msg::Point prev_goal_;
  double prev_distance_;

  geometry_msgs::msg::PoseStamped initial_pose_;
  void returnToInitialPose(void);

  // parameters
  bool return_to_init_;
  std::string robot_base_frame_;
  std::string map_frame_;
  //stops processing of next waypoint when locked
  std::condition_variable continue_condition_;
  //start_lock blocks main thread at start of processing if locked
  std::condition_variable start_condition_;
  //used to signal the final status of the last waypoint
  std::condition_variable goal_status_condition_;
  //how often to check status of plan execution
  double planner_frequency_ = 1.0;

  //this is used to drop out of a path mid journey
  bool continue_ = true;
  bool start_ = true;
  //the calculated path for coverage
  nav_msgs::msg::Path complete_path_;
  //the current pose index
  size_t current_pose_index_;
  //current feedback for goal
  rclcpp_action::ResultCode waypoint_status_;
  //mutexes for message signalling to primary path navigator
  
};
}  // namespace explore

#endif
