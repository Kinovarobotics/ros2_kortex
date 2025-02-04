#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "kortex_driver/msg/joint_angle.hpp"
#include "kortex_waypoint_controller/action/execute_joint_waypoints.hpp"

using ExecuteJointWaypoints = kortex_waypoint_controller::action::ExecuteJointWaypoints;
using GoalHandleExecuteJointWaypoints = rclcpp_action::ServerGoalHandle<ExecuteJointWaypoints>;

class ExecuteWaypointServer : public rclcpp::Node {
public:
    ExecuteWaypointServer() : Node("execute_waypoint_server") {
        this->action_server_ = rclcpp_action::create_server<ExecuteJointWaypoints>(
            this,
            "execute_joint_waypoints",
            std::bind(&ExecuteWaypointServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ExecuteWaypointServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ExecuteWaypointServer::handle_accepted, this, std::placeholders::_1)
        );
        this->action_client_ = rclcpp_action::create_client<ExecuteJointWaypoints>(
            this,
            "execute_joint_waypoints");
    }

private:
    rclcpp_action::Server<ExecuteJointWaypoints>::SharedPtr action_server_;
    rclcpp_action::Client<ExecuteJointWaypoints>::SharedPtr action_client_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteJointWaypoints::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received request to execute %lu waypoints", goal->waypoints.size());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteJointWaypoints> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Canceling waypoint execution");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteJointWaypoints> goal_handle) {
        std::thread{std::bind(&ExecuteWaypointServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleExecuteJointWaypoints> goal_handle) {
        auto feedback = std::make_shared<ExecuteJointWaypoints::Feedback>();
        auto result = std::make_shared<ExecuteJointWaypoints::Result>();

        auto goal = goal_handle->get_goal();

        if (!client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Kortex service not available!");
            result->success = false;
            result->message = "Service unavailable";
            goal_handle->abort(result);
            return;
        }

        auto request = std::make_shared<kortex_driver::srv::ExecuteWaypointTrajectory::Request>();

        // Convert ROS2 waypoints to Kortex WaypointList
        for (size_t i = 0; i < goal->waypoints.size(); ++i) {
            kortex_driver::msg::Waypoint wp;
            kortex_driver::msg::JointAngle angle;
            angle.joint_identifier = goal->waypoints[i].joint_identifier;
            angle.value = goal->waypoints[i].value;
            wp.joint_angles.push_back(angle);
            wp.duration = goal->durations[i];
            request->waypoints.waypoints.push_back(wp);
        }

        auto future = client_->async_send_request(request);

        // Process feedback
        while (rclcpp::ok()) {
            feedback->waypoint_index++;
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Waypoint execution completed");
            result->success = true;
            result->message = "Success";
            goal_handle->succeed(result);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Waypoint execution timed out");
            result->success = false;
            result->message = "Timeout";
            goal_handle->abort(result);
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExecuteWaypointServer>());
    rclcpp::shutdown();
    return 0;
}
