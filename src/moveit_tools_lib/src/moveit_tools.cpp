#include "moveit_tools_lib/moveit_tools.hpp"

#include <algorithm>  // std::clamp
#include <cmath>      // std::acos / std::cos / std::sin / std::fabs
#include <stdexcept>

// yaml-cpp 没有 tf2::Vector3 的内置转换器，这里提供特化：
// 约定 YAML 中以三元素序列 [x, y, z] 表示一个向量
namespace YAML {
    template<>
    struct convert<tf2::Vector3> {
        static bool decode(const Node& node, tf2::Vector3& out) {
            if (!node.IsSequence() || node.size() != 3) {
                return false;
            }
            out.setValue(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
            return true;
        }
    };
}

namespace moveit_tools{

    // ==================== PoseReceiveNode ====================

    PoseReceiveNode::PoseReceiveNode(const std::string& name, MotionQueuePtr& pose_queue)
        : rclcpp::Node(name), pose_queue_(std::move(pose_queue))
    {
        RCLCPP_INFO(this->get_logger(), "PoseReceiveNode has been initialized.");

        // 订阅目标位姿话题，收到后打印并推入队列
        this->pose_sub_ = this->create_subscription<PoseStamped>(
            "pose_topic", 10, [this](const PoseStamped::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received pose: [x: %f, y: %f, z: %f]",
                            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
                RCLCPP_INFO(this->get_logger(), "Received orientation: [x: %f, y: %f, z: %f, w: %f]",
                            msg->pose.orientation.x, msg->pose.orientation.y,
                            msg->pose.orientation.z, msg->pose.orientation.w);
                this->pose_queue_->push(*msg);
        });
    }

    PoseReceiveNode::~PoseReceiveNode()
    {
        RCLCPP_INFO(this->get_logger(), "PoseReceiveNode has been destroyed.");
    }

    // ==================== PoseGoalGraspTools ====================

    PoseGoalGraspTools::PoseGoalGraspTools(
        const std::shared_ptr<PoseReceiveNode>& node,
        MotionQueuePtr& pose_queue,
        const std::string& config_path
    ) : node_(node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_),
        pose_queue_(std::move(pose_queue))
    {
        RCLCPP_INFO(this->node_->get_logger(), "PoseGoalGraspTools has been initialized.");

        // 校验节点非空
        if (!node)
        {
            throw std::invalid_argument("node is null");
        }

        // 校验上层与节点共用同一个队列，避免出现“消费的队列与生产的队列不一致”
        if (node->getMotionPtr().get() != this->pose_queue_.get())
        {
            throw std::invalid_argument(
                "MoveItTool and PoseReceiveNode use different queues"
            );
        }

        YAML::Node yaml = YAML::LoadFile(config_path);

        this->config_.main_group_name  = yaml["main_group_name"].as<std::string>();
        this->config_.final_group_name = yaml["final_group_name"].as<std::string>();
        this->config_.main_vec  = yaml["main_vec"].as<double>();
        this->config_.main_acc  = yaml["main_acc"].as<double>();
        this->config_.final_vec = yaml["final_vec"].as<double>();
        this->config_.final_acc = yaml["final_acc"].as<double>();

        this->config_.execute            = yaml["execute"].as<bool>();
        this->config_.use_main_group     = yaml["use_main_group"].as<bool>();
        this->config_.use_cartesian      = yaml["cartesian_path"].as<bool>();
        this->config_.parallel_axis      = yaml["paraller_axis"].as<tf2::Vector3>();
        this->config_.min_fraction       = yaml["min_fraction"].as<double>();
        this->config_.eef_step           = yaml["eef_step"].as<double>();
        this->config_.approach_axis      = yaml["approach_axis"].as<tf2::Vector3>();
        this->config_.cartesian_distance = yaml["cartesian_ratio"].as<double>();
        this->config_.approach_distance  = yaml["approach_distance"].as<double>();

        // 创建主/末端规划组并设置速度、加速度缩放
        this->main_group_  = std::make_shared<MoveitGroupInterface>(this->node_, this->config_.main_group_name);
        this->final_group_ = std::make_shared<MoveitGroupInterface>(this->node_, this->config_.final_group_name);

        this->main_group_->setMaxVelocityScalingFactor(this->config_.main_vec);
        this->main_group_->setMaxAccelerationScalingFactor(this->config_.main_acc);
        this->final_group_->setMaxVelocityScalingFactor(this->config_.final_vec);
        this->final_group_->setMaxAccelerationScalingFactor(this->config_.final_acc);

        // 启动执行器线程处理节点回调
        this->executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        this->executor_->add_node(this->node_);
        this->executor_thread_ = std::thread([this]() { this->executor_->spin(); });

        // 启动工作线程消费位姿队列
        this->work_thread_ = std::thread([this]{ this->WorkLoop(); });
    }

    PoseGoalGraspTools::~PoseGoalGraspTools()
    {
        // 关闭队列，唤醒并让工作线程退出 waitPop
        this->node_->closeMotionQueue();

        // 停止执行器并回收执行器线程
        this->executor_->cancel();
        if (this->executor_thread_.joinable())
        {
            this->executor_thread_.join();
        }

        // 回收工作线程
        if (this->work_thread_.joinable())
        {
            this->work_thread_.join();
        }
        RCLCPP_INFO(this->node_->get_logger(), "PoseGoalGraspTools has been destroyed.");
    }

    bool PoseGoalGraspTools::namedGoal(
        const std::string& name,
        const bool execute,
        const bool use_main_group
    ){
        // 选择规划组，设置起点为当前状态与命名目标，然后规划/执行
        const MoveitGroupInterfacePtr& interface = use_main_group ? this->main_group_ : this->final_group_;
        interface->setStartStateToCurrentState();
        interface->setNamedTarget(name);
        return this->plan_or_execute(interface, execute);
    }

    bool PoseGoalGraspTools::jointGoal(
        const std::vector<double>& joints,
        const bool execute,
        const bool use_main_group
    ){
        const MoveitGroupInterfacePtr& interface = use_main_group ? this->main_group_ : this->final_group_;
        interface->setStartStateToCurrentState();
        interface->setJointValueTarget(joints);
        return this->plan_or_execute(interface, execute);
    }

    bool PoseGoalGraspTools::plan_or_execute(
        const MoveitGroupInterfacePtr& interface,
        bool execute
    ){
        MoveitGroupInterface::Plan plan;

        const bool ok = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!ok)
        {
            return false;
        }
        if (execute)
        {
            return (interface->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        return true;  // 仅规划，规划成功
    }

    void PoseGoalGraspTools::WorkLoop()
    {
        PoseStamped pose_target;

        // 阻塞式消费队列，直到队列关闭
        while (this->pose_queue_->waitPop(pose_target))
        {
            if (!poseGoal(
                pose_target,
                this->config_.execute,
                this->config_.use_main_group,
                this->config_.use_cartesian,
                this->config_.parallel_axis,
                this->config_.min_fraction,
                this->config_.eef_step,
                this->config_.approach_axis,
                this->config_.cartesian_distance,
                this->config_.approach_distance
            ))
            {
                RCLCPP_ERROR(this->node_->get_logger(), "规划失败");
            }
        }
    }

    tf2::Matrix3x3 PoseGoalGraspTools::compute_final_group_matrix(
        const tf2::Vector3& base2finalGroup,
        const tf2::Vector3& base2target
    ){
        auto vec_finalGroup = base2finalGroup.normalized();
        auto vec_target_norm = base2target.normalized();

        auto dot = std::clamp(vec_finalGroup.dot(vec_target_norm), -1.0, 1.0);

        // 若夹角为钝角，取目标轴反向，保证走最短旋转（对齐到同侧）
        auto vec_target = (dot > 0) ? vec_target_norm : -vec_target_norm;

        dot = std::clamp(vec_finalGroup.dot(vec_target), -1.0, 1.0);

        // 已基本平行，无需旋转
        if (std::fabs(dot - 1) < 1e-6)
        {
            return tf2::Matrix3x3::getIdentity();
        }

        // 旋转轴为两向量叉乘方向
        tf2::Vector3 k = vec_finalGroup.cross(vec_target).normalized();
        return this->Rodriguez_compute_mat(k, dot);
    }

    tf2::Matrix3x3 PoseGoalGraspTools::Rodriguez_compute_mat(
        const tf2::Vector3& k,
        const double dot
    ){
        const double theta = std::acos(dot);
        const double cosT = std::cos(theta);
        const double sinT = std::sin(theta);
        const double omc = 1.0 - cosT;  // one minus cos
        const double kx = k.x(), ky = k.y(), kz = k.z();

        // 罗德里格斯旋转公式展开为旋转矩阵
        tf2::Matrix3x3 R;
        R.setValue(
            cosT + kx*kx*omc,     kx*ky*omc - kz*sinT,  kx*kz*omc + ky*sinT,
            ky*kx*omc + kz*sinT,  cosT + ky*ky*omc,     ky*kz*omc - kx*sinT,
            kz*kx*omc - ky*sinT,  kz*ky*omc + kx*sinT,  cosT + kz*kz*omc
        );

        return R;
    }

    PoseStamped PoseGoalGraspTools::get_real_target_pose(
        const PoseStamped& target_pose,
        const TransformStamped& final_group_pose,
        const tf2::Matrix3x3& trans_matrix
    ){
        tf2::Quaternion q1;
        tf2::convert(final_group_pose.transform.rotation, q1);

        tf2::Matrix3x3 original_mat(q1);

        // 用对齐旋转左乘末端当前姿态，得到修正后的目标姿态
        tf2::Matrix3x3 target_pose_mat = trans_matrix * original_mat;

        tf2::Quaternion q2;
        target_pose_mat.getRotation(q2);

        auto real_target_pose = target_pose;  // 位置沿用目标，姿态用修正结果
        real_target_pose.pose.orientation.w = q2.w();
        real_target_pose.pose.orientation.x = q2.x();
        real_target_pose.pose.orientation.y = q2.y();
        real_target_pose.pose.orientation.z = q2.z();

        return real_target_pose;
    }

    void PoseGoalGraspTools::compute_pre_pose(
        PoseStamped& target_pose,
        PoseStamped& pre_target_pose,
        const tf2::Vector3& approach_axis,
        const double cartesian_distance,
        const double approach_distance
    ){
        // 目标位姿相对 base 的变换
        tf2::Transform T_base_obj;
        tf2::convert(target_pose.pose, T_base_obj);

        // approach_axis 是物体系下的“方向向量”。
        tf2::Vector3 offset_in_base   = T_base_obj.getBasis() * (approach_axis * cartesian_distance);
        tf2::Vector3 approach_in_base = T_base_obj.getBasis() * (approach_axis * approach_distance);

        pre_target_pose = target_pose;

        // 预抓取点：沿逼近方向后退 cartesian_distance
        pre_target_pose.pose.position.x -= offset_in_base.x();
        pre_target_pose.pose.position.y -= offset_in_base.y();
        pre_target_pose.pose.position.z -= offset_in_base.z();

        // 最终逼近点：在后退基础上再沿逼近方向前进 approach_distance
        target_pose.pose.position.x = target_pose.pose.position.x - offset_in_base.x() + approach_in_base.x();
        target_pose.pose.position.y = target_pose.pose.position.y - offset_in_base.y() + approach_in_base.y();
        target_pose.pose.position.z = target_pose.pose.position.z - offset_in_base.z() + approach_in_base.z();
    }

    TransformStamped PoseGoalGraspTools::tf_transform_arm(
        const std::string& source_name,
        const std::string& target_name
    ){
        TransformStamped source_in_target;

        try{
            source_in_target = this->tf_buffer_.lookupTransform(
                target_name,
                source_name,
                tf2::TimePointZero,
                tf2::durationFromSec(0.5)
            );
        } catch (const tf2::TransformException& ex){
            RCLCPP_ERROR(this->node_->get_logger(), "TF转换坐标系失败： %s", ex.what());
        }

        return source_in_target;
    }

    PoseStamped PoseGoalGraspTools::tf_transform_pose(
        const PoseStamped& source_pose,
        const std::string& target_name
    ){
        PoseStamped source_pose_in_target;
        try{
            source_pose_in_target = this->tf_buffer_.transform(
                source_pose,
                target_name,
                tf2::durationFromSec(0.5)
            );
        } catch (const tf2::TransformException& ex){
            RCLCPP_ERROR(this->node_->get_logger(), "TF转换姿态失败： %s", ex.what());
        }

        return source_pose_in_target;
    }

    tf2::Vector3 PoseGoalGraspTools::get_axis_in_base(
        const PoseStamped& pose,
        const tf2::Vector3& parallel_axis
    ){
        tf2::Quaternion q;
        tf2::convert(pose.pose.orientation, q);

        tf2::Matrix3x3 mat(q);

        // 根据选择轴确定索引：x->0, y->1, z->2
        int axis_ind = 0;
        if (parallel_axis.getX() != 0){
            axis_ind = 0;
        } else if (parallel_axis.getY() != 0){
            axis_ind = 1;
        } else if (parallel_axis.getZ() != 0){
            axis_ind = 2;
        }

        return mat.getColumn(axis_ind);
    }

    tf2::Vector3 PoseGoalGraspTools::get_axis_in_base(
        const TransformStamped& transform,
        const tf2::Vector3& parallel_axis
    ){
        tf2::Quaternion q;
        tf2::convert(transform.transform.rotation, q);

        tf2::Matrix3x3 mat(q);

        int axis_ind = 0;
        if (parallel_axis.getX() != 0){
            axis_ind = 0;
        } else if (parallel_axis.getY() != 0){
            axis_ind = 1;
        } else if (parallel_axis.getZ() != 0){
            axis_ind = 2;
        }

        return mat.getColumn(axis_ind);
    }

    bool PoseGoalGraspTools::plan_or_execute_cartesian_path(
        const PoseStamped& target_pose,
        const PoseStamped& pre_target_pose,
        const MoveitGroupInterfacePtr& interface,
        const bool execute,
        const double min_fraction,
        const double eef_step
    ){
        // 第一步：从当前状态规划到预抓取点
        interface->setStartStateToCurrentState();
        interface->setPoseTarget(pre_target_pose);

        MoveitGroupInterface::Plan plan;
        const bool ok = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        // 第二步：由预抓取点到最终目标点的笛卡尔直线路径
        std::vector<Pose> way_points{pre_target_pose.pose, target_pose.pose};
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double fraction = interface->computeCartesianPath(
            way_points, eef_step, 0.0, trajectory
        );

        // 仅规划：返回可达性判断
        if (!execute)
        {
            return ok && (fraction >= min_fraction);
        }

        // 执行：两段规划都成功且笛卡尔覆盖率达标才依次执行
        if (ok && (fraction >= min_fraction))
        {
            const bool execute_1 = (interface->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            const bool execute_2 = (interface->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
            return execute_1 && execute_2;
        }

        RCLCPP_WARN(this->node_->get_logger(), "规划失败，笛卡尔系数为：%.2f%%", fraction * 100);
        return false;
    }

    bool PoseGoalGraspTools::poseGoal(
        const PoseStamped& target,
        const bool execute,
        const bool use_main_group,
        const bool use_cartesian,
        const tf2::Vector3& parallel_axis,
        const double min_fraction,
        const double eef_step,
        const tf2::Vector3& approach_axis,
        const double cartesian_distance,
        const double approach_distance
    ){
        // 选择规划组
        const MoveitGroupInterfacePtr& interface = use_main_group ? this->main_group_ : this->final_group_;

        // 1. 将目标位姿变换到 base_link 坐标系
        auto target_pose_in_base = this->tf_transform_pose(target, "base_link");

        // 2. 查询末端(link_6)在 base_link 下的位姿
        auto final_group_in_base = this->tf_transform_arm("link_6", "base_link");

        // 3. 分别取目标与末端的“对齐轴”在 base 下的方向
        auto target_parallel_axis = this->get_axis_in_base(target_pose_in_base, parallel_axis);
        auto final_group_parallel_axis = this->get_axis_in_base(final_group_in_base, parallel_axis);

        // 4. 计算把末端轴旋转对齐到目标轴的旋转矩阵
        auto final_group_to_target_mat = this->compute_final_group_matrix(
            final_group_parallel_axis, target_parallel_axis
        );

        // 5. 用对齐矩阵修正末端目标姿态
        auto real_target_pose = this->get_real_target_pose(
            target_pose_in_base, final_group_in_base, final_group_to_target_mat
        );

        // 6. 计算预抓取点与最终逼近点
        PoseStamped pre_target_pose;
        this->compute_pre_pose(
            real_target_pose, pre_target_pose, approach_axis, cartesian_distance, approach_distance
        );

        // 7. 规划/执行
        if (use_cartesian)
        {
            return plan_or_execute_cartesian_path(
                real_target_pose, pre_target_pose, interface, execute, min_fraction, eef_step
            );
        }

        // 非笛卡尔：直接规划到最终目标位姿
        interface->setStartStateToCurrentState();
        interface->setPoseTarget(real_target_pose);
        return this->plan_or_execute(interface, execute);
    }
}
