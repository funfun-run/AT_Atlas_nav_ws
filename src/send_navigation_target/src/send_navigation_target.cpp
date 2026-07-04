/**
 * @file send_navigation_target.cpp
 * @brief 导航目标发送节点 —— 通过服务调用逐点触发导航
 *
 * 工作流程：
 *   1. 启动后等待 "go_next_waypoint" 服务调用（std_srvs::Trigger）
 *   2. 收到调用后，从内部硬编码的导航点列表取当前索引对应的坐标
 *   3. 通过 NavigateToPose action client 发送给 Nav2 导航堆栈
 *   4. 同步阻塞等待导航结果（到达 / 失败 / 取消）
 *   5. 返回结果（success + message）给调用方，索引 +1
 *   6. 调用方再次调用服务时，前往下一个点
 *
 * 导航点列表在构造函数中硬编码，修改路线时直接改 waypoints_ 即可。
 *
 * 线程模型（MultiThreadedExecutor）：
 *   - Executor 线程池：处理所有 ROS2 回调，不低于 2 个线程
 *   - 服务线程：handle_go_next() 在某个线程中阻塞等待，不占满整个线程池
 *   - action 回调线程：result_callback / feedback_callback / goal_response_callback
 *     在其他线程中执行，触发后通过 condition_variable 通知服务线程
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"   // Nav2 导航 action 定义
#include "std_srvs/srv/trigger.hpp"                // 空请求 / 简单响应的标准服务
#include <vector>
#include <mutex>
#include <condition_variable>
#include <chrono>

class SendNavigationTarget : public rclcpp::Node
{
private:
    // ============ 类型别名 ============
    using NavigateToPose = nav2_msgs::action::NavigateToPose;

    // ============ ROS2 通信对象 ============
    /// Action 回调的可重入回调组 —— 独立于默认互斥组，使 action 回调能与
    /// 阻塞中的 handle_go_next() 并发执行。不设此组则所有回调挤在默认互斥组，
    /// handle_go_next 的 cv_.wait() 会阻塞所有其他回调。
    rclcpp::CallbackGroup::SharedPtr action_callback_group_;

    /// Nav2 导航 action client —— 负责向导航堆栈发送目标点
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

    /// "go_next_waypoint" 服务端 —— 外部（FSM）每次调用即前往下一个点
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    // ============ 导航点管理（硬编码） ============
    /// 预定义的导航点列表（x, y），按调用顺序依次执行，方向默认
    const std::vector<std::pair<double, double>> waypoints_ = {
        {0.0, 0.0},   // 点1
        {1.5, 1.5},   // 点2
        {0.0, 0.0},   // 点3
        {1.5, 1.5},   // 点4
        {0.0, 0.0},   // 点5
        {1.5, 1.5},   // 点6
        {0.0, 0.0},   // 点7
        {2.0, 3.0},   // 点8
        {5.0, 2.0},   // 点9
    };

    /// 当前即将前往的点索引（0-based），每次服务调用成功后 +1
    size_t current_index_ = 0;

    // ============ 线程同步：服务线程 ↔ action 回调线程 ============
    /// 服务调用是同步阻塞的：handle_go_next() 发送 goal 后必须等待导航完成
    /// 才能返回响应。action 回调在不同线程中触发，因此需要跨线程等待机制。
    std::mutex mtx_;
    std::condition_variable cv_;
    bool goal_done_ = false;     ///< 当前 goal 是否已出结果（成功/失败/取消）
    bool goal_success_ = false;  ///< 当前 goal 是否成功到达

    // ============ Action 回调（在 action client 内部线程执行）============

    /**
     * @brief 导航最终结果回调
     *
     * 无论成功、失败还是被取消，此回调都会被触发。
     * 设置同步标志后唤醒在 handle_go_next() 中等待的服务线程。
     *
     * @param result 包装的导航结果，包含 result_code 和 NavigateToPose::Result
     */
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
                RCLCPP_ERROR(get_logger(), "未知结果码: %d", static_cast<int>(result.code));
                goal_success_ = false;
                break;
        }
        goal_done_ = true;
        cv_.notify_one();  // 唤醒阻塞在 handle_go_next() 中的服务线程
    }

    /**
     * @brief Goal 接受/拒绝回调
     *
     * async_send_goal 之后，Nav2 action server 首先判断是否接受这个 goal。
     * 如果 goal_handle 为 nullptr，说明被拒绝（通常是机器人未定位或 TF 异常）。
     * 此回调保证 handle_go_next 不会在 goal 被静默拒绝时永远阻塞。
     */
    void goal_response_callback(
        std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(get_logger(),
                         "Goal 被服务器拒绝！可能原因：机器人未定位、TF 异常、costmap 未就绪");
            std::lock_guard<std::mutex> lock(mtx_);
            goal_success_ = false;
            goal_done_ = true;
            cv_.notify_one();
        } else {
            RCLCPP_INFO(get_logger(), "Goal 被服务器接受，等待导航完成...");
        }
    }

    /**
     * @brief 导航进度反馈回调
     *
     * 定期触发，打印剩余距离用于监控导航过程。
     *
     * @param feedback 包含 distance_remaining（剩余距离，米）等信息
     */
    void feedback_callback(
        std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>>,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(),
                    "  [进度] 剩余 %.2f 米", feedback->distance_remaining);
    }

    // ============ 服务回调（在 ROS2 服务独立线程中执行）============

    /**
     * @brief "go_next_waypoint" 服务处理函数
     *
     * 每次外部调用此服务时执行以下步骤：
     *   1. 检查是否还有未执行的点
     *   2. 从 waypoints_ 取当前索引对应的坐标
     *   3. 构造 NavigateToPose::Goal 并发送
     *   4. 阻塞等待导航完成（不阻塞主线程 spin）
     *   5. 索引自增，返回成功/失败
     *
     * @param request  空请求（Trigger 服务无请求字段）
     * @param response 包含 success（是否到达）和 message（描述信息）
     */
    void handle_go_next(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // --- 检查是否已走完所有导航点 ---
        if (current_index_ >= waypoints_.size()) {
            response->success = false;
            response->message = "所有导航点已完成";
            RCLCPP_WARN(get_logger(),
                        "收到 go_next 请求，但 %zu 个点已全部执行完毕",
                        waypoints_.size());
            return;
        }

        // --- 取当前目标点坐标 ---
        auto [x, y] = waypoints_[current_index_];
        size_t idx = current_index_;  // 保存索引用于日志（wait 之后 current_index_ 会变）
        RCLCPP_INFO(get_logger(),
                    "===== 前往第 %zu / %zu 个点: (%.2f, %.2f) =====",
                    idx + 1, waypoints_.size(), x, y);

        // --- 构造 NavigateToPose Goal ---
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";       // 所有目标均在 map 坐标系下
        goal_msg.pose.header.stamp = this->now();    // 时间戳用当前时刻
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.position.z = 0.0;
        // 方向设为无旋转（单位四元数），表示只关心位置，不约束最终朝向
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        // --- 重置同步标志（上锁保护写入） ---
        {
            std::lock_guard<std::mutex> lock(mtx_);
            goal_done_ = false;
            goal_success_ = false;
        }

        // --- 绑定回调并异步发送 goal ---
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

        // --- 同步等待导航结果（带超时保护）---
        // 阻塞当前服务线程，直到 result_callback 或 goal_response_callback
        // （goal 被拒时）设置 goal_done_。超时 300 秒作为兜底，防止永久阻塞。
        // 主线程的 spin 不受影响，因为服务回调运行在独立的线程池中。
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
                goal_done_ = true;  // 标记完成，防止延迟回调再触发混乱
            }
            success = goal_success_;
        }

        // --- 导航结束，推进索引 ---
        current_index_++;

        // --- 填充服务响应 ---
        response->success = success;
        if (success) {
            response->message = "成功到达";
            RCLCPP_INFO(get_logger(), "第 %zu 个点: 到达 ✓", idx + 1);
        } else {
            response->message = "导航失败";
            RCLCPP_ERROR(get_logger(), "第 %zu 个点: 失败 ✗", idx + 1);
        }
    }

public:
    /**
     * @brief 构造函数 —— 初始化 action client 和 service server
     *
     * @param name 节点名称
     */
    explicit SendNavigationTarget(const std::string & name)
        : Node(name)
    {
        // 为 action 回调创建独立的可重入回调组
        // 必须与 handle_go_next（默认互斥组）分离，否则 cv_.wait() 阻塞服务回调时，
        // action 的 goal_response / feedback / result 回调全部得不到执行。
        this->action_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // 创建 action client，指定使用可重入回调组
        this->action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose", action_callback_group_);

        // 等待 Nav2 action server 上线（阻塞最多 10 秒）
        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(get_logger(),
                         "navigate_to_pose action server 未就绪，节点将退出");
            rclcpp::shutdown();
            return;
        }

        // 创建 "go_next_waypoint" 服务，
        // 外部（competition_fsm）每次调用触发前往下一个导航点
        this->service_ = this->create_service<std_srvs::srv::Trigger>(
            "go_next_waypoint",
            std::bind(&SendNavigationTarget::handle_go_next,
                      this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(),
                    "初始化完成，已加载 %zu 个导航点，等待服务调用...",
                    waypoints_.size());
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SendNavigationTarget>("send_navigation_target");

    // 关键：必须用多线程 executor，而不是 rclcpp::spin()（单线程）。
    // 单线程 executor 下，handle_go_next() 的 cv_.wait() 会阻塞唯一的 spin 线程，
    // 导致 action 回调（goal_response / feedback / result）永远得不到处理。
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
