#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"  // Correct include

class SendCustomPathNode : public rclcpp::Node
{
public:
    using FollowPath = nav2_msgs::action::FollowPath;
    using NavigateToPose = nav2_msgs::action::NavigateToPose;  // Add this line to define NavigateToPose
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;  // Define handle for NavigateToPose

    SendCustomPathNode()
        : Node("send_custom_path_node"), path_sent_(false),initial_goal_sent_(false)
    {
        // Define action client for FollowPath
        follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

        // Define action client for NavigateToPose
        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Wait for action servers to be available
        RCLCPP_INFO(this->get_logger(), "Waiting for the action servers...");
        while (!follow_path_client_->wait_for_action_server(std::chrono::seconds(5)) ||
               !nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the action servers...");
        }

        // Subscribe to the robot's pose (e.g., from AMCL)
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/current_pose", 10,
            std::bind(&SendCustomPathNode::poseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to /current_pose.");
    }

private:
    rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;  // Action client for NavigateToPose
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool current_pose_received_ = false;
    bool path_sent_ = false;
    bool initial_goal_sent_ =false;

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (path_sent_) {
            return;  // Path already sent, do nothing
        }
        if (initial_goal_sent_){
            return;
        }

        // Log to verify the pose is received
        RCLCPP_INFO(this->get_logger(), "Current pose received: [x: %f, y: %f]",
                    msg->pose.position.x, msg->pose.position.y);

        // Store current pose
        current_pose_.header = msg->header;
        current_pose_.pose = msg->pose;
        current_pose_received_ = true;

        sendInitialGoal();  // Send the initial goal after receiving the first pose
        initial_goal_sent_ = true;
    }

    void sendInitialGoal()
    {
        if (!current_pose_received_ || path_sent_) {
            return;  // Ensure goal and path are sent only once
        }

        // Target pose: 1 meter ahead in the x direction
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "map";
        target_pose.pose.position.x = current_pose_.pose.position.x + 1.0;
        target_pose.pose.position.y = current_pose_.pose.position.y;
        target_pose.pose.position.z = current_pose_.pose.position.z;
        target_pose.pose.orientation = current_pose_.pose.orientation;

        // Log the target position
        RCLCPP_INFO(this->get_logger(), "Sending initial goal to position: [x: %f, y: %f]",
                    target_pose.pose.position.x, target_pose.pose.position.y);

        // Create goal message for NavigateToPose action server
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = target_pose;

        // Define the goal options with a result callback
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&SendCustomPathNode::onGoalReached, this, std::placeholders::_1);

        // Send the goal to the NavigateToPose action server
        RCLCPP_INFO(this->get_logger(), "Sending goal to NavigateToPose action server...");
        nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void onGoalReached(const GoalHandleNavigateToPose::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Initial goal reached. Now sending the path...");
            sendPath();  // Call the sendPath() function once the goal is reached
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to reach the initial goal.");
        }
    }

    void sendPath()
    {
        if (!current_pose_received_ || path_sent_) {
            return;  // Ensure path is sent only once
        }

        // Target pose: 5 meters ahead in the x direction
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "map";
        target_pose.pose.position.x = current_pose_.pose.position.x + 5.0;
        target_pose.pose.position.y = current_pose_.pose.position.y;
        target_pose.pose.position.z = current_pose_.pose.position.z;
        target_pose.pose.orientation = current_pose_.pose.orientation;

        // Log target position
        RCLCPP_INFO(this->get_logger(), "Generating path to position: [x: %f, y: %f]",
                    target_pose.pose.position.x, target_pose.pose.position.y);

        // Create path message
        auto path_msg = std::make_shared<nav_msgs::msg::Path>();
        path_msg->header.frame_id = "map";
        path_msg->header.stamp = this->now();

        // Add intermediate points between current and target pose
        int num_points = 10;  // Number of intermediate points
        for (int i = 0; i <= num_points; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->now();
            pose.pose.position.x = current_pose_.pose.position.x + (target_pose.pose.position.x - current_pose_.pose.position.x) * i / num_points;
            pose.pose.position.y = current_pose_.pose.position.y;  // Keep y constant
            pose.pose.position.z = current_pose_.pose.position.z;  // Keep z constant
            pose.pose.orientation = current_pose_.pose.orientation;  // Keep orientation constant
            path_msg->poses.push_back(pose);
        }

        // Send the path to the FollowPath action server
        auto goal_msg = FollowPath::Goal();
        goal_msg.path = *path_msg;

        RCLCPP_INFO(this->get_logger(), "Sending path to FollowPath action server...");

        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&SendCustomPathNode::resultCallback, this, std::placeholders::_1);

        follow_path_client_->async_send_goal(goal_msg, send_goal_options);

        // Mark the path as sent
        path_sent_ = true;
    }

    void resultCallback(const GoalHandleFollowPath::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Path followed successfully!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Path following aborted.");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Path following canceled.");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
            break;
        }
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SendCustomPathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
