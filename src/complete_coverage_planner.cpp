#include "complete_coverage_planner/complete_coverage_planner.hpp"
#include "complete_coverage_planner/spiral_stc.hpp"
#include <thread>
#include <mutex>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace complete_coverage_planner
{
CompleteCoverage::CompleteCoverage()
  : Node("complete_coverage_node")
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
{
  this->declare_parameter<float>("robot_radius", 0.1);
  this->declare_parameter<float>("tool_radius", 0.1);
  this->declare_parameter<bool>("return_to_init", false);
  this->declare_parameter<bool>("visualize", false);

  bool visualize;
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("visualize", visualize);

  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, "navigate_to_pose");

  if (visualize) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("complete_coverage_planner/"
                                                                     "path"
                                                                     "s",
                                                                     10);
  }
  // Subscription to pause or resume complete coverage path, does nothing until plan started
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "complete_coverage/resume", 10,
      std::bind(&CompleteCoverage::resumeCallback, this, std::placeholders::_1));
  // Topic to stop and start processing new path
  stop_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "complete_coverage/start", 10,
      std::bind(&CompleteCoverage::start, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");
  map_frame_ = costmap_client_.getGlobalFrameID();

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame_, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.pose.position.x = transformStamped.transform.translation.x;
      initial_pose_.pose.position.y = transformStamped.transform.translation.y;
      initial_pose_.pose.orientation = transformStamped.transform.rotation;
      initial_pose_.header = transformStamped.header;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }
  try{
    while(1) {
      if(start_) makePlan();
      else {
        std::mutex lk;
        std::unique_lock<std::mutex> ulk(lk);
        start_condition_.wait(ulk);
      }
    }
  }catch(const std::exception &e){
    RCLCPP_ERROR(logger_, "Exception while planning %s",e.what());
  }

}

void CompleteCoverage::makePlan()
{
  SpiralSTC spiral_path_builder;
  
  spiral_path_builder.configure(this,"complete_coverage_planner",costmap_client_.getCostmap(),map_frame_);
  //This creates the complete coverage path
  nav_msgs::msg::Path complete_path = spiral_path_builder.createPlan(initial_pose_);
  //Now we have to feed the waypoints in one at a time so our "smart" path planner doesn't shortcut tracks
  
  RCLCPP_DEBUG(logger_, "total poses %d",complete_path.poses.size());
  //std::vector<geometry_msgs::msg::PoseStamped> poses;
  auto nav_poses = nav2_msgs::action::NavigateThroughPoses::Goal();
  for(auto waypoint: complete_path.poses)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.pose = waypoint.pose;
    goal.header.frame_id = map_frame_;
    goal.header.stamp = this->now();
    nav2_msgs::action::NavigateToPose::Goal navgoal;
    navgoal.pose = goal;
   // nav_poses.poses.push_back(goal);
  std::mutex lk;
  std::unique_lock<std::mutex> ulk(lk);
  //if stopped mid path
  if(!start_){
    RCLCPP_DEBUG(logger_, "stop set, stopping");
    stop();
    return;
  }
  //pause feature
  while(!continue_){
    RCLCPP_DEBUG(logger_, "pause set, waiting for continue");
    continue_condition_.wait(ulk);
  }
  //save for debug logging
  RCLCPP_DEBUG(logger_, "Sending goals to move base nav2");
  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
    [this](const NavigationGoalHandle::WrappedResult& result) {
      reachedGoal(result);
    };
    send_goal_options.feedback_callback =
      [this]( std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>,
       std::shared_ptr<const nav2_msgs::action::NavigateToPose_Feedback_<std::allocator<void>>> fb){
          feedbackCallback(fb);
       };
    move_base_client_->async_send_goal(navgoal, send_goal_options);

    //now wait for some response from the action, hopefully goal completed
    while(waypoint_status_ == rclcpp_action::ResultCode::UNKNOWN)
    {
      continue_condition_.wait(ulk);
    }
    switch(waypoint_status_)
    {
      //continue processing waypoints
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
        case rclcpp_action::ResultCode::CANCELED:
        case rclcpp_action::ResultCode::UNKNOWN:
        //quite and move to start if requested
          stop();
        //wait for new start to replan and start again
          start_ = false;
          return;
    }
  }
  if(return_to_init_)returnToInitialPose();
}
void CompleteCoverage::feedbackCallback(std::shared_ptr<const nav2_msgs::action::NavigateToPose_Feedback_<std::allocator<void>>> fb) 
{
    RCLCPP_INFO(logger_,"Resume set to %lf",fb->distance_remaining);
}
void CompleteCoverage::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{

    continue_=msg->data;
    RCLCPP_INFO(logger_,"Resume set to %s",continue_ ?"true":"false");
    //if this is an resume, notify any waits
    if(continue_) continue_condition_.notify_all();
}

void CompleteCoverage::start(const std_msgs::msg::Bool::SharedPtr msg)
{
  start_ = msg->data;
  std::string starting;
  if(start_) starting = "starting.";
  else starting = "stopping.";
  RCLCPP_INFO(logger_, "Exploration %s",start_?"true":"false");
  //if starting signal
  if(start_){
    start_condition_.notify_all();
  }
}
CompleteCoverage::~CompleteCoverage()
{
  stop();
}



void CompleteCoverage::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.pose.position;
  goal.pose.pose.orientation = initial_pose_.pose.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void CompleteCoverage::reachedGoal(const NavigationGoalHandle::WrappedResult& result)
{
  waypoint_status_ = result.code;
  switch (result.code) {

    RCLCPP_INFO(logger_, "Status received for point: x:%lf,y:%lf,z:%lf",prev_goal_.x,prev_goal_.y,prev_goal_.z);
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger_, "Goal was successful");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(logger_, "Goal was aborted");
      // If it was aborted probably because we've found another frontier goal,
      // so just return and don't make plan again
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
    //OK, so since we are in an unknown state but we did get a response, conservatively cancel the operation
      waypoint_status_ =  rclcpp_action::ResultCode::CANCELED;
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
}


void CompleteCoverage::stop()
{
  RCLCPP_INFO(logger_, "Exploration stopped.");
  if (return_to_init_) {
    returnToInitialPose();
  }
}

}  // namespace complete_coverage

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<complete_coverage_planner::CompleteCoverage>()); 
  rclcpp::shutdown();
  return 0;
}
