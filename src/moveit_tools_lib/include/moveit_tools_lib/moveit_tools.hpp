#ifndef MOVEIT_TOOLS_HPP_
#define MOVEIT_TOOLS_HPP_


#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "safe_thread_queue/safe_thread_queue.hpp"

using MoveitGroupInterface = moveit::planning_interface::MoveGroupInterface;
using MoveitGroupInterfacePtr = std::shared_ptr<MoveitGroupInterface>;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using MotionQueue = tools::ThreadSafeQueue<PoseStamped>;
using MotionQueuePtr = std::shared_ptr<MotionQueue>;

namespace moveit_tools{

class PoseReceiveNode : public rclcpp::Node
{
public:
    explicit PoseReceiveNode(const std::string& name, MotionQueuePtr& pose_queue);
    ~PoseReceiveNode();

    MotionQueuePtr getMotionPtr() const { return pose_queue_; }

    void closeMotionQueue() { pose_queue_->close(); }

private:
    rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
    MotionQueuePtr pose_queue_;
};


class PoseGoalGraspTools
{
public:

    PoseGoalGraspTools(
        const std::shared_ptr<PoseReceiveNode>& node,
        MotionQueuePtr& pose_queue,
        const std::string& config_path
    );
    ~PoseGoalGraspTools();

    bool namedGoal(const std::string& name, bool execute = true, const bool use_main_group = false);

    bool jointGoal(const std::vector<double>& joints, const bool execute = true, const bool use_main_group = false);

    bool poseGoal(
        const PoseStamped& target,
        const bool exexute = true,
        const bool use_main_group = true,
        const bool use_cartesian = true,
        const tf2::Vector3& parallel_axis = {0, 0, 1},
        const double min_fracton = 0.95,
        const double eef_step = 0.01,
        const tf2::Vector3& approch_axis = {1, 0, 0},
        const double cartesian_distance = 0.15,
        const double approach_distance
    );

    PoseGoalGraspTools(const PoseGoalGraspTools&) = delete;
    PoseGoalGraspTools& operator=(const PoseGoalGraspTools&) = delete;

private:
    void WorkLoop();

    bool plan_or_execute(
        const MoveitGroupInterfacePtr& interface,
        bool execute
    );

    bool plan_or_execute_cartesian_path(
        const PoseStamped& target_pose,
        const PoseStamped& pre_target_pose,
        const MoveitGroupInterfacePtr& interface,
        const bool execute,
        const double min_fraction,
        const double min_eff
    );

    void compute_pre_pose(
        PoseStamped& target_pose,
        PoseStamped& pre_target_pose,
        const tf2::Vector3& approach_aixs,
        const double cartesian_distance,
        const double approach_distance
    );

    tf2::Matrix3x3 compute_final_group_matrix(
        const tf2::Vector3& base2finalGroup,
        const tf2::Vector3& base2target
    );

    tf2::Matrix3x3 Rodriguez_compute_mat(
        const tf2::Vector3& k,
        const double dot
    );

    PoseStamped get_real_target_pose(
        const PoseStamped& target_pose,
        const TransformStamped& fianl_group_pose,
        const tf2::Matrix3x3& trans_matrix
    );

    TransformStamped tf_transform_arm
    (
        const std::string& source_name,
        const std::string& target_name
    );

    PoseStamped tf_transform_pose
    (
        const PoseStamped& source_pose,
        const std::string& target_name   
    );

    tf2::Vector3 get_axis_in_base
    (
        const PoseStamped& pose,
        const tf2::Vector3& parallel_axis
    );

    tf2::Vector3 get_axis_in_base
    (
        const TransformStamped& transform,
        const tf2::Vector3& parallel_axis
    );

private:
    // 私有成员属性
    std::shared_ptr<PoseReceiveNode> node_;
    MoveitGroupInterfacePtr main_group_, final_group_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    MotionQueuePtr pose_queue_;
    rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;
    std::thread executor_thread_;
    std::thread work_thread_;
    YAML::Node config_;
};

}

#endif // MOVEIT_TOOLS_HPP_