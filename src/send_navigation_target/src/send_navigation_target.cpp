/**
 * @file send_navigation_target.cpp
 * @brief 导航目标发送节点 —— 总控通过服务指定目标点标识（A/B/C/D）
 *
 * 工作流程：
 *   1. 启动后等待 "navigate_to_target" 服务调用
 *   2. 收到调用后，根据 waypoint_id（A/B/C/D）查表得坐标
 *   3. 通过 NavigateToPose action client 发送给 Nav2 导航堆栈
 *   4. 同步阻塞等待导航结果（到达 / 失败 / 取消）
 *   5. 返回结果（success + message）给总控
 *
 * 总控用法示例：
 *   ros2 service call /navigate_to_target send_navigation_target/srv/NavigateToTarget \
 *     "{waypoint_id: 'B'}"
 *
 * 线程模型（MultiThreadedExecutor + Reentrant 回调组）：
 *   - 服务回调在默认互斥组中阻塞等待
 *   - Action 回调在独立可重入组中并发执行
 *   - condition_variable 桥接两个线程组
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "send_navigation_target/srv/navigate_to_target.hpp"
#include <map>
#include <string>
#include <mutex>
#include <condition_variable>
#include <chrono>

class SendNavigationTarget : public rclcpp::Node
{
private:
    // ============ 类型别名 ============
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using NavigateToTarget = send_navigation_target::srv::NavigateToTarget;

    // ============ ROS2 通信对象 ============
    /// Action 回调的可重入回调组 —— 独立于默认互斥组，使 action 回调能与
    /// 阻塞中的服务回调并发执行。不设此组则所有回调挤在默认互斥组，
    /// cv_.wait() 阻塞服务回调的同时也阻塞了所有 action 回调。
    rclcpp::CallbackGroup::SharedPtr action_callback_group_;

    /// Nav2 导航 action client
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

    /// "navigate_to_target" 服务端 —— 总控每次调用指定目标点标识
    rclcpp::Service<NavigateToTarget>::SharedPtr service_;

    // ============ 导航点坐标表 ============
    /// 目标点标识 → 坐标（x, y），修改路线时只改这里即可
    const std::map<std::string, std::pair<double, double>> waypoint_map_ = {
        {"A", {0.0, 0.0}},
        {"B", {1.5, 1.5}},
        {"C", {2.0, 3.0}},
        {"D", {5.0, 2.0}},
    };

    // ============ 线程同步：服务线程 ↔ action 回调线程 ============
    std::mutex mtx_;
    std::condition_variable cv_;
    bool goal_done_ = false;
    bool goal_success_ = false;

    // ============ Action 回调（在 action client 内部线程执行）============

    void result_callback(
        const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(), "导航到达！");
                goal_success_ = true;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "导航被中止");
                goal_success_ = false;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(get_logger(), "导航被取消");
                goal_success_ = false;
                break;
            default:
                RCLCPP_ERROR(get_logger(), "未知结果码: %d",
                             static_cast<int>(result.code));
                goal_success_ = false;
                break;
        }
        goal_done_ = true;
        cv_.notify_one();
    }

    void goal_response_callback(
        std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(get_logger(),
                         "Goal 被服务器拒绝！可能原因：机器人未定位、TF 异常、"
                         "costmap 未就绪");
            std::lock_guard<std::mutex> lock(mtx_);
            goal_success_ = false;
            goal_done_ = true;
            cv_.notify_one();
        } else {
            RCLCPP_INFO(get_logger(), "Goal 被服务器接受，等待导航完成...");
        }
    }

    void feedback_callback(
        std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>>,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(),
                    "  [进度] 剩余 %.2f 米", feedback->distance_remaining);
    }

    // ============ 服务回调（在 ROS2 服务线程中执行）============

    /**
     * @brief "navigate_to_target" 服务处理函数
     *
     * 总控每次调用时指定 waypoint_id（A/B/C/D），
     * 查表得坐标，发送 Nav2 goal，同步等待结果。
     *
     * @param request  包含 waypoint_id 字段
     * @param response 返回 success + message
     */
    void handle_navigate_to_target(
        const std::shared_ptr<NavigateToTarget::Request> request,
        std::shared_ptr<NavigateToTarget::Response> response)
    {
        const auto & wid = request->waypoint_id;

        // --- 查表 ---
        auto it = waypoint_map_.find(wid);
        if (it == waypoint_map_.end()) {
            response->success = false;
            response->message = "未知的目标点标识: " + wid;
            RCLCPP_ERROR(get_logger(),
                         "收到未知 waypoint_id: '%s'，已知: A B C D", wid.c_str());
            return;
        }

        auto [x, y] = it->second;
        RCLCPP_INFO(get_logger(),
                    "===== 前往目标点 %s: (%.2f, %.2f) =====", wid.c_str(), x, y);

        // --- 构造 Goal ---
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.position.z = 0.0;
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        // --- 重置同步标志 ---
        {
            std::lock_guard<std::mutex> lock(mtx_);
            goal_done_ = false;
            goal_success_ = false;
        }

        // --- 绑定回调并异步发送 ---
        using std::placeholders::_1;
        using std::placeholders::_2;
        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        opts.goal_response_callback =
            std::bind(&SendNavigationTarget::goal_response_callback, this, _1);
        opts.feedback_callback =
            std::bind(&SendNavigationTarget::feedback_callback, this, _1, _2);
        opts.result_callback =
            std::bind(&SendNavigationTarget::result_callback, this, _1);
        this->action_client_->async_send_goal(goal_msg, opts);

        // --- 同步等待（带超时） ---
        constexpr auto kTimeout = std::chrono::seconds(300);
        bool success;
        {
            std::unique_lock<std::mutex> lock(mtx_);
            bool finished = cv_.wait_for(lock, kTimeout,
                                         [this] { return goal_done_; });
            if (!finished) {
                RCLCPP_ERROR(get_logger(), "导航超时（%lld 秒），放弃等待",
                             static_cast<long long>(kTimeout.count()));
                goal_success_ = false;
                goal_done_ = true;
            }
            success = goal_success_;
        }

        // --- 填充响应 ---
        response->success = success;
        if (success) {
            response->message = "成功到达 " + wid;
            RCLCPP_INFO(get_logger(), "目标点 %s: 到达 ✓", wid.c_str());
        } else {
            response->message = "导航失败";
            RCLCPP_ERROR(get_logger(), "目标点 %s: 失败 ✗", wid.c_str());
        }
    }

public:
    explicit SendNavigationTarget(const std::string & name)
        : Node(name)
    {
        // 为 action 回调创建独立可重入回调组，与 handle_navigate_to_target
        // 所在的默认互斥组分离，避免 cv_.wait() 阻塞所有回调。
        this->action_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        this->action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose", action_callback_group_);

        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(get_logger(),
                         "navigate_to_pose action server 未就绪，节点将退出");
            rclcpp::shutdown();
            return;
        }

        this->service_ = this->create_service<NavigateToTarget>(
            "navigate_to_target",
            std::bind(&SendNavigationTarget::handle_navigate_to_target,
                      this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(),
                    "初始化完成，已加载 %zu 个目标点（A B C D），等待服务调用...",
                    waypoint_map_.size());
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SendNavigationTarget>("send_navigation_target");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
