#ifndef MOVEIT_TOOLS_HPP_
#define MOVEIT_TOOLS_HPP_


#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <memory>
#include <string>
#include <vector>

using MoveitGroupInterface = moveit::planning_interface::MoveGroupInterface;

namespace moveit_tools{

struct pose
{
    double x_, y_, z_;
    tf2::Quaternion q_;
  
    void init(const double& x, 
            const double& y,
            const double& z,
            const double& roll,
            const double& pitch,
            const double& yaw)
    {
        x_ = x;
        y_ = y;
        z_ = z;
        q_.setRPY(roll, pitch, yaw);
        q_.normalize();
    }
};

class Moveit
{
public:
    // 构造函数
    Moveit(std::shared_ptr<rclcpp::Node>& node, 
                const float& vec_scale = 1.0f, 
                const float& acc_scale = 1.0f);
    // 析构函数
    ~Moveit();
    
    // 禁止拷贝
    Moveit(const Moveit&) = delete;
    Moveit& operator=(const Moveit&) = delete;
    
    // 外部接口函数
    // nameGoal
    bool namedGoal(const std::string& name, const bool& execute = true);

    // jointGoal
    bool jointGoal(const std::vector<double>& joint, const bool& execute = true);

    // poseGoal
    bool poseGoal(const std::shared_ptr<pose>& pose,
                  const bool& execute = true, 
                  const bool& certesian_path = true,
                  const double& eef_step = 0.01,
                  const double& min_fraction = 0.95,
                  const double& retreat_distance = 0.1);

private:
    // 私有工具函数
    
    // 规划执行工具函数
    bool PlanAndExecute(const std::shared_ptr<MoveitGroupInterface>& interface, const bool execute = true);

    // pre_grasp计算
    geometry_msgs::msg::PoseStamped computePreGrasp(const geometry_msgs::msg::PoseStamped& grasp, const double& retreat_distance = 0.1);

    // 笛卡尔规划，避免抓取过程中不必要的碰撞
    bool cartesionApproachAndGrasp(const std::shared_ptr<MoveitGroupInterface>& interface,
                                   const geometry_msgs::msg::Pose& pre_grasp_pose,
                                   const geometry_msgs::msg::Pose& grasp_pose,
                                   const bool& execute = true,
                                   const double& eef_step = 0.01,
                                   const double& min_fraction = 0.95);


    // 私有属性
    std::shared_ptr<rclcpp::Node> node_;                    // 节点
    std::shared_ptr<MoveitGroupInterface> arm_;             // 机械臂规划组
    std::shared_ptr<MoveitGroupInterface> gripper_;         // 夹爪规划组

    rclcpp::executors::SingleThreadedExecutor executor_;     // 单进程执行器
    std::thread spin_thread_;                               // ros2循环执行线程
};

}

#endif // MOVEIT_TOOLS_HPP_