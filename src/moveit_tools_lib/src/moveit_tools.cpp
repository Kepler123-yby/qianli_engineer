#include "moveit_tools_lib/moveit_tools.hpp"

namespace moveit_tools{
    
    PoseReceiveNode::PoseReceiveNode(const std::string& name, MotionQueuePtr& pose_queue) : rclcpp::Node(name), pose_queue_(std::move(pose_queue))
    {
        RCLCPP_INFO(this->get_logger(), "PoseReceiveNode has been initialized.");
        
        this->pose_sub_ = this->create_subscription<PoseStamped>(
            "pose_topic", 10, [this](const PoseStamped::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received pose: [x: %f, y: %f, z: %f]", 
                            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
                RCLCPP_INFO(this->get_logger(), "Received orientation: [x: %f, y: %f, z: %f, w: %f]", 
                            msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
                this->pose_queue_->push(*msg);
        });
    }

    PoseReceiveNode::~PoseReceiveNode()
    {
        RCLCPP_INFO(this->get_logger(), "PoseReceiveNode has been destroyed.");
    }

    PoseGoalGraspTools::PoseGoalGraspTools(
        const std::shared_ptr<PoseReceiveNode>& node,
        MotionQueuePtr& pose_queue,
        const std::string& config_path
    ) : node_(node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_), pose_queue_(std::move(pose_queue))
    {
        RCLCPP_INFO(this->node_->get_logger(), "PoseGoalGraspTools has been initialized.");

        if (!node)
        {
            throw std::invalid_argument("node is null");
        }

        if (node->getMotionPtr().get() != this->pose_queue_.get())
        {
            throw std::invalid_argument(
                "MoveItTool and PoseReceiveNode use different queues"
            );
        }

        this->config_ = YAML::Load(config_path);

        std::string main_group_name = this->config_["main_group_name"].as<std::string>();
        std::string final_group_name = this->config_["final_group_name"].as<std::string>();
        this->main_group_ = std::make_shared<MoveitGroupInterface>(main_group_name);
        this->final_group_ = std::make_shared<MoveitGroupInterface>(final_group_name);

        double main_vec = this->config_["main_vec"].as<double>();
        double main_acc = this->config_["main_acc"].as<double>();
        double final_vec = this->config_["final_vec"].as<double>();
        double final_acc = this->config_["final_acc"].as<double>();

        this->main_group_->setMaxVelocityScalingFactor(main_vec);
        this->main_group_->setMaxAccelerationScalingFactor(main_acc);

        this->final_group_->setMaxVelocityScalingFactor(final_vec);
        this->final_group_->setMaxAccelerationScalingFactor(final_acc);
        
        this->executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        this->executor_->add_node(this->node_);
        this->executor_thread_ = std::thread([this]() { this->executor_->spin(); });
    
        this->work_thread_ = std::thread(
            [this]{
                this->WorkLoop();
            }
        );
    }

    PoseGoalGraspTools::~PoseGoalGraspTools()
    {
        this->node_->closeMotionQueue();

        this->executor_->cancel();
        if (this->executor_thread_.joinable()) {
            this->executor_thread_.join();
        }

        if (this->work_thread_.joinable())
        {
            work_thread_.join();
        }
        RCLCPP_INFO(this->node_->get_logger(), "PoseGoalGraspTools has been destroyed.");
    }

    bool PoseGoalGraspTools::namedGoal(
        const std::string& name,
        const bool execute,
        const bool use_main_group
    ){
        if (use_main_group)
        {
            this->main_group_->setStartStateToCurrentState();
            this->main_group_->setNamedTarget(name);
            return this->plan_or_execute(main_group_, execute);
        } else {
            this->final_group_->setStartStateToCurrentState();
            this->final_group_->setNamedTarget(name);
            return this->plan_or_execute(final_group_, execute);
        }
        
    };

    bool PoseGoalGraspTools::jointGoal(
        const std::vector<double>& joints,
        const bool execute,
        const bool use_main_group
    ){
        if (use_main_group){
            this->main_group_->setStartStateToCurrentState();
            this->main_group_->setJointValueTarget(joints);
            this->plan_or_execute(main_group_, execute);
        } else {
            this->final_group_->setStartStateToCurrentState();
            this->final_group_->setJointValueTarget(joints);
            this->plan_or_execute(final_group_, execute);
        }
    }

    bool PoseGoalGraspTools::plan_or_execute(
        const MoveitGroupInterfacePtr& interface,
        bool execute
    ){
        MoveitGroupInterface::Plan plan;

        bool ok = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (execute && ok)
        {
            return (interface->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }

        if (!execute)
        {
            return ok;

        }
    }

    void PoseGoalGraspTools::WorkLoop()
    {

        PoseStamped pose_target;

        while (this->pose_queue_->waitPop(pose_target))
        {
            std::string error;

            bool execute = this->config_["execute"].as<bool>();
            bool use_main_group = this->config_["use_main_group"].as<bool>();
            bool use_cartesian_path = this->config_["cartesian_path"].as<bool>();
            tf2::Vector3 paraller_axis = this->config_["paraller_axis"].as<tf2::Vector3>();
            double min_fraction = this->config_["min_fraction"].as<double>();
            double eef_step = this->config_["eef_step"].as<double>();
            tf2::Vector3 approach_axis = this->config_["approach_axis"].as<tf2::Vector3>();
            double cartesian_distance = this->config_["cartesian_ratio"].as<double>();
            double approach_distance = this->config_["approach_distance"].as<double>();

            if (!poseGoal(
                pose_target,
                execute,
                use_main_group,
                use_cartesian_path,
                paraller_axis,
                min_fraction,
                eef_step,
                approach_axis,
                cartesian_distance,
                approach_distance
            ))
            {
                RCLCPP_ERROR(this->node_->get_logger(), "规划失败：%s", error.c_str());
            }
        }
    }

    tf2::Matrix3x3 PoseGoalGraspTools::compute_final_group_matrix(
        const tf2::Vector3& base2finalGroup,
        const tf2::Vector3& base2target
    ){
        auto vec_finaGroup = base2finalGroup.normalized();
        auto vec_target_norm = base2target.normalized();
        auto dot = std::clamp(vec_finaGroup.dot(vec_target_norm), 1.0, -1.0);

        auto vec_target = (dot > 0) ? vec_target_norm : -vec_target_norm;
        
        dot = std::clamp(vec_finaGroup.dot(vec_target), 1.0, -1.0);
        if (std::fabs(dot - 1) < 1e-6)
        {
            return tf2::Matrix3x3::getIdentity();
        }

        tf2::Vector3 k = vec_finaGroup.cross(vec_target).normalized();
        return this->Rodriguez_compute_mat(k, dot);
    }

    tf2::Matrix3x3 PoseGoalGraspTools::Rodriguez_compute_mat(
        const tf2::Vector3& k,
        const double dot
    ){
        double theta = std::acos(dot);
        double cosT = std::cos(dot);
        double sinT = std::sin(dot);
        double omc = 1.0 - cosT;
        double kx = k.x(), ky = k.y(), kz = k.z();

        tf2::Matrix3x3 R;
        R.setValue(
            cosT + kx*kx*omc, ky*kx*omc - kz*sinT, kx*kz*omc + ky*sinT,
            ky*kx*omc + kz*sinT, cosT + ky*ky*omc, ky*kz*omc - kx*sinT,
            kz*kx*omc - ky*sinT, kz*ky*omc + kx*sinT, cosT + kz*kz*omc
        );
    }

    PoseStamped PoseGoalGraspTools::get_real_target_pose(
        const PoseStamped& target_pose,
        const TransformStamped& fianl_group_pose,
        const tf2::Matrix3x3& trans_matrix
    ){
        tf2::Quaternion q1;
        tf2::convert(fianl_group_pose, q1);

        tf2::Matrix3x3 original_mat(q1);

        tf2::Matrix3x3 target_pose_mat = trans_matrix * original_mat;
        
        tf2::Quaternion q2;
        target_pose_mat.getRotation(q2);

        auto real_target_pose = target_pose;

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

        tf2::Transform T_base_obj;
        tf2::convert(target_pose.pose, T_base_obj);

        tf2::Transform T_obj_base;
        T_obj_base = T_base_obj.inverse();

        tf2::Vector3 offest_in_obj = approach_axis * cartesian_distance;
        tf2::Vector3 approach_in_obj = approach_axis * approach_distance;

        tf2::Vector3 offset_in_base = T_obj_base * offest_in_obj;
        tf2::Vector3 approach_in_base = T_obj_base * approach_in_obj;
        
        auto pre_target_pose = target_pose;

        pre_target_pose.pose.position.x -= offset_in_base.x();
        pre_target_pose.pose.position.y -= offset_in_base.y();
        pre_target_pose.pose.position.z -= offset_in_base.z();

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

    tf2::Vector3 get_axis_in_base(
        const PoseStamped& pose, 
        const tf2::Vector3& parallel_axis
    ){
        tf2::Quaternion q;
        tf2::convert(pose.pose.orientation, q);

        tf2::Matrix3x3 mat(q);
        
        int axis_ind;

        if (parallel_axis.getX() != 0){
            axis_ind = 0;
        } else if (parallel_axis.getY() != 0){
            axis_ind = 1;
        } else if (parallel_axis.getZ() != 0){
            axis_ind = 2;
        }

        return mat.getRow(axis_ind);
    }

    tf2::Vector3 get_axis_in_base(
        const TransformStamped& transform,
        const tf2::Vector3& parallel_axis
    ){
        tf2::Quaternion q;
        tf2::convert(transform.transform.rotation, q);

        tf2::Matrix3x3 mat(q);
        
        int axis_ind;

        if (parallel_axis.getX() != 0){
            axis_ind = 0;
        } else if (parallel_axis.getY() != 0){
            axis_ind = 1;
        } else if (parallel_axis.getZ() != 0){
            axis_ind = 2;
        }

        return mat.getRow(axis_ind);
    }

    bool PoseGoalGraspTools::plan_or_execute_cartesian_path(
        const PoseStamped& target_pose,
        const PoseStamped& pre_target_pose,
        const MoveitGroupInterfacePtr& interface,
        const bool execute,
        const double min_fraction,
        const double eff_step
    ){
        interface->setStartStateToCurrentState();
        interface->setPoseTarget(pre_target_pose);
        
        MoveitGroupInterface::Plan plan;

        bool ok = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        std::vector<Pose> way_points{pre_target_pose.pose, target_pose.pose};

        moveit_msgs::msg::RobotTrajectory trajectory;

        double fractoin = interface->computeCartesianPath(
            way_points, eff_step, 0.0, trajectory
        );

        if (!execute)
        {
            return ok && (fractoin >= min_fraction);
        } else if (execute){
            if (ok && (fractoin >= min_fraction)){
                bool execute_1 = (interface->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                bool execute_2 = (interface->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
                return execute_1 && execute_2;
            } else {
                RCLCPP_WARN(this->node_->get_logger(), "规划失败，笛卡尔系数为：%.2f%%", fractoin*100);
                return ok && (fractoin >= min_fraction);
            }
        }
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
        auto target_pose_in_base = this->tf_transform_pose(
            target,
            "base_link"
        );

        auto final_group_in_base = this->tf_transform_arm(
            "link_6",
            "base_link"
        );

        auto target_parallel_axis = this->get_axis_in_base(
            target_pose_in_base,
            parallel_axis
        );

        auto final_group_parallel_axis = this->get_axis_in_base(
            final_group_in_base,
            parallel_axis
        );

        auto final_group_to_target_mat = this->compute_final_group_matrix(
            final_group_parallel_axis,
            target_parallel_axis
        );

        auto real_target_pose = this->get_real_target_pose(
            target_pose_in_base,
            final_group_in_base,
            final_group_to_target_mat
        );

        PoseStamped pre_target_pose;

        this->compute_pre_pose(
            real_target_pose,
            pre_target_pose,
            approach_axis,
            cartesian_distance,
            approach_distance
        );

        return plan_or_execute_cartesian_path(
            real_target_pose,
            pre_target_pose,
            this->main_group_,
            execute,
            min_fraction,
            eef_step
        );
    }
}