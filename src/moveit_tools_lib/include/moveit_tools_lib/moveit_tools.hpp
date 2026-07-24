#ifndef MOVEIT_TOOLS_HPP_
#define MOVEIT_TOOLS_HPP_

// ---- tf2 相关：四元数/变换/旋转矩阵，以及消息与坐标系监听 ----
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

// ---- ROS2 / MoveIt / 消息 / 标准库 ----
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "safe_thread_queue/safe_thread_queue.hpp"

// ---- 类型别名：统一简化 MoveIt / 几何消息 / 线程安全队列的书写 ----
using MoveitGroupInterface = moveit::planning_interface::MoveGroupInterface;
using MoveitGroupInterfacePtr = std::shared_ptr<MoveitGroupInterface>;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using MotionQueue = tools::ThreadSafeQueue<PoseStamped>;
using MotionQueuePtr = std::shared_ptr<MotionQueue>;

namespace moveit_tools{

/**
 * @brief 目标位姿接收节点
 *
 * 订阅 "pose_topic" 话题，将收到的目标位姿推入线程安全队列，
 * 供 PoseGoalGraspTools 的工作线程消费。
 */
class PoseReceiveNode : public rclcpp::Node
{
public:
    explicit PoseReceiveNode(const std::string& name, MotionQueuePtr& pose_queue);
    ~PoseReceiveNode();

    /// 获取内部使用的运动队列指针（用于校验上层是否共用同一队列）
    MotionQueuePtr getMotionPtr() const { return pose_queue_; }

    /// 关闭队列：唤醒并终止所有阻塞在 waitPop 上的消费者
    void closeMotionQueue() { pose_queue_->close(); }

private:
    rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;  ///< 位姿订阅器
    MotionQueuePtr pose_queue_;                              ///< 目标位姿队列
};


/**
 * @brief 抓取位姿对齐 + 规划执行工具
 *
 * 从队列中取出目标位姿，先把末端执行器的指定轴对齐到目标轴，
 * 再计算预抓取点，最后通过关节空间或笛卡尔路径完成规划/执行。
 * 内部维护一个独立执行器线程与一个工作线程。
 */
class PoseGoalGraspTools
{
public:

    PoseGoalGraspTools(
        const std::shared_ptr<PoseReceiveNode>& node,
        MotionQueuePtr& pose_queue,
        const std::string& config_path
    );
    ~PoseGoalGraspTools();

    /// 运动到配置文件中的命名目标（如 "home"）
    bool namedGoal(const std::string& name, bool execute = true, const bool use_main_group = false);

    /// 运动到给定的关节角目标
    bool jointGoal(const std::vector<double>& joints, const bool execute = true, const bool use_main_group = false);

    /**
     * @brief 面向抓取的位姿目标：对齐 + 逼近 + 规划/执行
     * @param target             目标位姿（任意坐标系，内部会变换到 base_link）
     * @param execute            true 则执行，false 仅做可达性规划
     * @param use_main_group     true 用主规划组，false 用末端规划组
     * @param use_cartesian      true 走笛卡尔逼近路径，false 走普通位姿规划
     * @param parallel_axis      需要对齐的“平行轴”（末端系/目标系下的选择轴）
     * @param min_fraction       笛卡尔路径最小覆盖率阈值
     * @param eef_step           笛卡尔插补步长（米）
     * @param approach_axis      逼近方向（物体坐标系下）
     * @param cartesian_distance 预抓取相对目标的后退距离（米）
     * @param approach_distance  最终逼近深度（米）
     */
    bool poseGoal(
        const PoseStamped& target,
        const bool execute = true,
        const bool use_main_group = true,
        const bool use_cartesian = true,
        const tf2::Vector3& parallel_axis = {0, 0, 1},
        const double min_fraction = 0.95,
        const double eef_step = 0.01,
        const tf2::Vector3& approach_axis = {1, 0, 0},
        const double cartesian_distance = 0.15,
        const double approach_distance = 0.05
    );

    // 禁止拷贝：内部持有线程与队列，拷贝语义无意义
    PoseGoalGraspTools(const PoseGoalGraspTools&) = delete;
    PoseGoalGraspTools& operator=(const PoseGoalGraspTools&) = delete;

private:
    /**
     * @brief 抓取运动参数缓存
     *
     * 构造时从 YAML 解析一次并缓存，避免 WorkLoop 每次循环重复读取。
     */
    struct GraspConfig
    {
        std::string main_group_name;   ///< 主规划组名
        std::string final_group_name;  ///< 末端规划组名
        double main_vec = 0.1;         ///< 主组速度缩放
        double main_acc = 0.1;         ///< 主组加速度缩放
        double final_vec = 0.1;        ///< 末端组速度缩放
        double final_acc = 0.1;        ///< 末端组加速度缩放

        bool execute = true;                            ///< 是否执行
        bool use_main_group = true;                     ///< 是否使用主规划组
        bool use_cartesian = true;                      ///< 是否走笛卡尔路径
        tf2::Vector3 parallel_axis{0, 0, 1};            ///< 对齐轴
        double min_fraction = 0.95;                     ///< 笛卡尔最小覆盖率
        double eef_step = 0.01;                         ///< 笛卡尔插补步长
        tf2::Vector3 approach_axis{1, 0, 0};            ///< 逼近方向
        double cartesian_distance = 0.15;               ///< 预抓取后退距离
        double approach_distance = 0.05;                ///< 最终逼近深度
    };

    /// 工作线程主循环：从队列取位姿并调用 poseGoal
    void WorkLoop();

    /// 通用规划/执行：对已设置好目标的规划组进行 plan（可选 execute）
    bool plan_or_execute(
        const MoveitGroupInterfacePtr& interface,
        bool execute
    );

    /// 先到预抓取点，再以笛卡尔直线逼近到目标点
    bool plan_or_execute_cartesian_path(
        const PoseStamped& target_pose,
        const PoseStamped& pre_target_pose,
        const MoveitGroupInterfacePtr& interface,
        const bool execute,
        const double min_fraction,
        const double eef_step
    );

    /// 由目标位姿计算预抓取点(pre_target_pose)与最终逼近点(target_pose)
    void compute_pre_pose(
        PoseStamped& target_pose,
        PoseStamped& pre_target_pose,
        const tf2::Vector3& approach_axis,
        const double cartesian_distance,
        const double approach_distance
    );

    /// 计算把末端轴(base2finalGroup)旋转对齐到目标轴(base2target)的旋转矩阵
    tf2::Matrix3x3 compute_final_group_matrix(
        const tf2::Vector3& base2finalGroup,
        const tf2::Vector3& base2target
    );

    /// 罗德里格斯公式：绕单位轴 k 旋转 acos(dot) 弧度，返回旋转矩阵
    tf2::Matrix3x3 Rodriguez_compute_mat(
        const tf2::Vector3& k,
        const double dot
    );

    /// 用对齐旋转矩阵修正目标姿态，得到末端真实应达到的目标位姿
    PoseStamped get_real_target_pose(
        const PoseStamped& target_pose,
        const TransformStamped& final_group_pose,
        const tf2::Matrix3x3& trans_matrix
    );

    /// 查询 source_name 坐标系在 target_name 坐标系下的变换
    TransformStamped tf_transform_arm(
        const std::string& source_name,
        const std::string& target_name
    );

    /// 将 source_pose 变换到 target_name 坐标系
    PoseStamped tf_transform_pose(
        const PoseStamped& source_pose,
        const std::string& target_name
    );

    /// 提取位姿中指定轴在 base 系下的方向向量
    tf2::Vector3 get_axis_in_base(
        const PoseStamped& pose,
        const tf2::Vector3& parallel_axis
    );

    /// 提取变换中指定轴在 base 系下的方向向量（重载）
    tf2::Vector3 get_axis_in_base(
        const TransformStamped& transform,
        const tf2::Vector3& parallel_axis
    );

private:
    // ---- 私有成员属性 ----
    std::shared_ptr<PoseReceiveNode> node_;                       ///< 位姿接收节点
    MoveitGroupInterfacePtr main_group_, final_group_;            ///< 主/末端规划组
    tf2_ros::Buffer tf_buffer_;                                   ///< TF 缓存
    tf2_ros::TransformListener tf_listener_;                      ///< TF 监听器
    MotionQueuePtr pose_queue_;                                   ///< 目标位姿队列
    rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;  ///< 单线程执行器
    std::thread executor_thread_;                                 ///< 执行器线程
    std::thread work_thread_;                                     ///< 工作线程
    GraspConfig config_;                                          ///< 缓存后的抓取参数
};

}

#endif // MOVEIT_TOOLS_HPP_
