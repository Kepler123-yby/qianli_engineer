#include "moveit_tools_lib/moveit_tools.hpp"

namespace moveit_tools{
    
    Moveit::Moveit(std::shared_ptr<rclcpp::Node>& node, 
                                        const float& vec_scale, 
                                        const float& acc_scale)
    {
        this->node_ = std::shared_ptr<rclcpp::Node>(node);                                 // 初始化node
        this->arm_ = std::make_shared<MoveitGroupInterface>(this->node_, "arm");            // 创建arm规划组
        this->gripper_ = std::make_shared<MoveitGroupInterface>(this->node_, "gripper");    // 创建gripper规划组

        // 设置规划组参数
        this->arm_->setMaxVelocityScalingFactor(vec_scale);           // 速度倍数：1.0
        this->gripper_->setMaxVelocityScalingFactor(vec_scale);

        this->arm_->setMaxAccelerationScalingFactor(acc_scale);
        this->gripper_->setMaxAccelerationScalingFactor(acc_scale);   // 加速度倍率：1.0

        // 将节点挂载到executor上
        this->executor_.add_node(this->node_);
        this->spin_thread_ = std::thread([this] () {this->executor_.spin();});
    }

    Moveit::~Moveit()
    {
        this->executor_.cancel();
        if (this->spin_thread_.joinable()){
            spin_thread_.join();
        }
    }

    bool Moveit::PlanAndExecute(const std::shared_ptr<MoveitGroupInterface>& interface, const bool execute)
    {
        // 创建规划
        MoveitGroupInterface::Plan plan;
        
        bool ok = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (ok && execute) {
            interface->execute(plan);
        }
        
        return ok;
    }

    geometry_msgs::msg::PoseStamped Moveit::computePreGrasp(const geometry_msgs::msg::PoseStamped& grasp, const double& retreat_distance)
    {
        // 把grasp的旋转坐标的四元数转换为可供tf2计算的q
        tf2::Quaternion q;
        tf2::fromMsg(grasp.pose.orientation, q);

        // 定义靠近目标的坐标轴x
        tf2::Vector3 approch_axis(1, 0, 0);

        // 将局部变换转到base_link
        tf2::Vector3 approch_world = tf2::quatRotate(q, approch_axis);

        geometry_msgs::msg::PoseStamped pre_grasp = grasp;

        // 计算回退距离在base_link坐标系下的长度
        pre_grasp.pose.position.x -= approch_world.x() * retreat_distance;
        pre_grasp.pose.position.y -= approch_world.y() * retreat_distance;
        pre_grasp.pose.position.z -= approch_world.z() * retreat_distance;

        return pre_grasp;
    }

    bool Moveit::cartesionApproachAndGrasp(const std::shared_ptr<MoveitGroupInterface>& interface,
                                           const geometry_msgs::msg::Pose& pre_grasp_pose,
                                           const geometry_msgs::msg::Pose& grasp_pose,
                                           const bool& execute,
                                           const double& eef_step,
                                           const double& min_fraction)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints{pre_grasp_pose, grasp_pose};

        moveit_msgs::msg::RobotTrajectory trajectory;

        double fraction = interface->computeCartesianPath(waypoints, eef_step, 0.0, trajectory);

        if (fraction < min_fraction)
        {
            RCLCPP_WARN(this->node_->get_logger(), "Grasp cartesian path incomplete: %.2f%%", fraction * 100.0);
            return false;
        } else {
            if (execute){
                bool ok = (interface->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
                return ok;
            } else {
                return true;
            }
        }
    }
    
    bool Moveit::namedGoal(const std::string& name, const bool& execute)
    {
        this->arm_->setStartStateToCurrentState();
        this->arm_->setNamedTarget(name);
        return this->PlanAndExecute(this->arm_, execute);
    }

    bool Moveit::jointGoal(const std::vector<double>& joint, const bool& execute)
    {
        this->arm_->setStartStateToCurrentState();
        this->arm_->setJointValueTarget(joint);
        return this->PlanAndExecute(this->arm_, execute);
    }

    bool Moveit::poseGoal(const std::shared_ptr<pose>& pose,
                          const bool& execute, 
                          const bool& certesian_path,
                          const double& eef_step,
                          const double& min_fraction,
                          const double& retreat_distance)
    {
        this->arm_->setStartStateToCurrentState();

        geometry_msgs::msg::PoseStamped grasp_pose;
        grasp_pose.header.frame_id = "base_link";
        grasp_pose.pose.position.x = pose->x_;
        grasp_pose.pose.position.y = pose->y_;
        grasp_pose.pose.position.z = pose->z_;
        grasp_pose.pose.orientation.w = pose->q_.getW();
        grasp_pose.pose.orientation.x = pose->q_.getX();
        grasp_pose.pose.orientation.y = pose->q_.getY();
        grasp_pose.pose.orientation.z = pose->q_.getZ();

        if (!certesian_path)
        {
            this->arm_->setPoseTarget(grasp_pose);
            return this->PlanAndExecute(this->arm_, execute);
        }

        // 获取pre_grasp_pose
        auto pre_grasp_pose = this->computePreGrasp(grasp_pose, retreat_distance);

        return this->cartesionApproachAndGrasp(this->arm_, pre_grasp_pose.pose, grasp_pose.pose, execute,  eef_step, min_fraction);
    }

}