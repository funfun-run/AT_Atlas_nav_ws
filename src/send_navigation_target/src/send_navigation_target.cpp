#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class SendNavigationTarget : public rclcpp::Node
{
	private:
		rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;

		void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
		{
			if (!goal_handle) {
				RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
			} else {
				RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
			}
		}
		
		void feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
		{
			RCLCPP_INFO(this->get_logger(), "Received feedback: %.2f m remaining", feedback->distance_remaining);
		}

		void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
		{
			switch (result.code) {
				case rclcpp_action::ResultCode::SUCCEEDED:
					RCLCPP_INFO(this->get_logger(), "Goal was succeeded!");
					break;
				case rclcpp_action::ResultCode::ABORTED:
					RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
					return;
				case rclcpp_action::ResultCode::CANCELED:
					RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
					return;
				default:
					RCLCPP_ERROR(this->get_logger(), "Unknown result code");
					return;
			}
			RCLCPP_INFO(this->get_logger(), "Navigation completed");
		}

		void send_navigation_target()
		{
			using namespace std::placeholders;
			
			if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
				RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
				rclcpp::shutdown();
				return;
			}

			auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
			
			goal_msg.pose.header.frame_id = "map";
			goal_msg.pose.header.stamp = this->now();
			
			goal_msg.pose.pose.position.x = 0.0;
			goal_msg.pose.pose.position.y = 0.0;
			goal_msg.pose.pose.position.z = 0.0;

			goal_msg.pose.pose.orientation.x = 0.0;
			goal_msg.pose.pose.orientation.y = 0.0;
			goal_msg.pose.pose.orientation.z = 0.0;
			goal_msg.pose.pose.orientation.w = 1.0;

			auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
			send_goal_options.goal_response_callback = std::bind(&SendNavigationTarget::goal_response_callback, this, _1);
			send_goal_options.feedback_callback = std::bind(&SendNavigationTarget::feedback_callback, this, _1, _2);
			send_goal_options.result_callback = std::bind(&SendNavigationTarget::result_callback, this, _1);
			this->action_client_->async_send_goal(goal_msg, send_goal_options);
		}
  	public:
    	SendNavigationTarget(std::string name) : Node(name)
    	{
			RCLCPP_INFO(this->get_logger(), "start 'send navigation target' node");
    	  	using namespace std::placeholders;

			this -> action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
		
			this->send_navigation_target();
		}
	
};

int main (int argc, char * argv[])
{
  	rclcpp::init(argc, argv);

  	auto node = std::make_shared<SendNavigationTarget>("send_navigation_target");
  
  	rclcpp::spin(node);
  
 	rclcpp::shutdown();
  
  	return 0;
}