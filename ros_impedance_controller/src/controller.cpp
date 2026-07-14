/**
 * @file controller.cpp
 * @author Gennaro Raiola, Michele Focchi (ROS2 port)
 * @brief ROS impedance controller ported to ros2_control (Humble).
 *
 * The control math (discrete and continuous PID + feed-forward effort) is
 * identical to the original ROS1 ros_control plugin. Only the middleware
 * layer (node handles, pub/sub, services, parameters, hardware interface
 * access) has been migrated to ROS2 / ros2_control.
 */

#include <ros_impedance_controller/controller.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <cstring>
#include <string>

namespace ros_impedance_controller {

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

Controller::Controller() {}
Controller::~Controller() {}

controller_interface::CallbackReturn Controller::on_init()
{
    try
    {
        // joints and per-joint gains are read in on_configure. Declare the
        // list parameter here; gains are declared dynamically once joint
        // names are known.
        auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
        auto_declare<bool>("pid_discrete_implementation", false);
        auto_declare<std::string>("robot_name", std::string());
        auto_declare<bool>("verbose", false);
    }
    catch (const std::exception & e)
    {
        fprintf(stderr, "Exception thrown during on_init with message: %s\n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Controller::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto node = get_node();
    RCLCPP_INFO_STREAM(node->get_logger(),
        cyan << "ROS_IMPEDANCE CONTROLLER: Initialize Ros Impedance Controller" << reset);

    joint_names_ = node->get_parameter("joints").as_string_array();
    if (joint_names_.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "No joints given in the namespace: %s.",
                     node->get_namespace());
        return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO_STREAM(node->get_logger(),
        green << "Found " << joint_names_.size() << " joints" << reset);

    discrete_implementation_ = node->get_parameter("pid_discrete_implementation").as_bool();
    robot_name_ = node->get_parameter("robot_name").as_string();
    if (discrete_implementation_)
        RCLCPP_INFO_STREAM(node->get_logger(),
            green << "Discrete implementation of PID control! (low the gains)" << reset);
    else
        RCLCPP_INFO_STREAM(node->get_logger(),
            green << "Continuous implementation of PID control!" << reset);

    const std::size_t n = joint_names_.size();

    // Resize the variables
    des_joint_positions_.setZero(n);
    integral_action_old_.setZero(n);
    des_joint_velocities_.setZero(n);
    des_joint_efforts_.setZero(n);
    des_joint_efforts_pids_.setZero(n);
    measured_joint_position_.setZero(n);
    measured_joint_velocity_.setZero(n);
    joint_positions_old_.setZero(n);
    error_.setZero(n);
    error1_.setZero(n);
    T_i.setZero(n);
    T_d.setZero(n);
    proportional_action_.setZero(n);
    integral_action_.setZero(n);
    derivative_action_.setZero(n);
    use_integral_action_.setZero(n);

    joint_type_.assign(n, "revolute");
    joint_p_gain_.resize(n);
    joint_i_gain_.resize(n);
    joint_d_gain_.resize(n);

    for (std::size_t i = 0; i < n; i++)
    {
        const std::string & jn = joint_names_[i];
        // Declare + read per-joint PID gains from parameters
        joint_p_gain_[i] = node->declare_parameter<double>("gains." + jn + ".p", -1.0);
        joint_i_gain_[i] = node->declare_parameter<double>("gains." + jn + ".i", -1.0);
        joint_d_gain_[i] = node->declare_parameter<double>("gains." + jn + ".d", -1.0);

        if (joint_p_gain_[i] < 0.0 || joint_i_gain_[i] < 0.0 || joint_d_gain_[i] < 0.0)
        {
            RCLCPP_ERROR(node->get_logger(),
                "Missing or negative PID gain for joint %s. All gains must be given and positive.",
                jn.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
        RCLCPP_DEBUG(node->get_logger(), "P value for joint %s is: %f", jn.c_str(), joint_p_gain_[i]);
        RCLCPP_DEBUG(node->get_logger(), "I value for joint %s is: %f", jn.c_str(), joint_i_gain_[i]);
        RCLCPP_DEBUG(node->get_logger(), "D value for joint %s is: %f", jn.c_str(), joint_d_gain_[i]);

        joint_type_[i] = node->declare_parameter<std::string>("joint_type." + jn, "revolute");
        // startup (home) position
        des_joint_positions_[i] = node->declare_parameter<double>("home." + jn, 0.0);

        if (joint_i_gain_[i] == 0)
        {
            use_integral_action_[i] = false;
            T_i[i] = 0;
        }
        else
        {
            use_integral_action_[i] = true;
            T_i[i] = joint_p_gain_[i] / joint_i_gain_[i];
        }
        T_d[i] = (joint_p_gain_[i] == 0) ? 0 : joint_d_gain_[i] / joint_p_gain_[i];
    }

    // Create subscriber (command topic)
    command_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        "/command", rclcpp::SystemDefaultsQoS(),
        std::bind(&Controller::commandCallback, this, std::placeholders::_1));

    // Create real-time publisher for the effort PID debug topic
    effort_pid_pub_raw_ = node->create_publisher<ros_impedance_controller::msg::EffortPid>(
        "effort_pid", rclcpp::SystemDefaultsQoS());
    effort_pid_pub_ = std::make_shared<
        realtime_tools::RealtimePublisher<ros_impedance_controller::msg::EffortPid>>(effort_pid_pub_raw_);

    // Create the PID set service
    set_pids_srv_ = node->create_service<ros_impedance_controller::srv::SetPids>(
        "/set_pids",
        std::bind(&Controller::setPidsCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize the command buffer with the home position
    Command initial;
    initial.des_joint_positions = des_joint_positions_;
    initial.des_joint_velocities = Eigen::VectorXd::Zero(n);
    initial.des_joint_efforts = Eigen::VectorXd::Zero(n);
    command_buffer_.writeFromNonRT(initial);

    RCLCPP_INFO_STREAM(node->get_logger(),
        cyan << "ROS_IMPEDANCE CONTROLLER: ROBOT NAME IS : " << robot_name_ << reset);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Controller::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(joint_names_.size());
    for (const auto & jn : joint_names_)
        conf.names.push_back(jn + "/" + hardware_interface::HW_IF_EFFORT);
    return conf;
}

controller_interface::InterfaceConfiguration
Controller::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(joint_names_.size() * 2);
    for (const auto & jn : joint_names_)
    {
        conf.names.push_back(jn + "/" + hardware_interface::HW_IF_POSITION);
        conf.names.push_back(jn + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return conf;
}

controller_interface::CallbackReturn Controller::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_DEBUG(get_node()->get_logger(), "Starting Controller");
    // Reset integral term on activation
    integral_action_old_.setZero(joint_names_.size());
    integral_action_.setZero(joint_names_.size());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Controller::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_DEBUG(get_node()->get_logger(), "Stopping Controller");
    return controller_interface::CallbackReturn::SUCCESS;
}

void Controller::setPidsCallback(
    const std::shared_ptr<ros_impedance_controller::srv::SetPids::Request> req,
    std::shared_ptr<ros_impedance_controller::srv::SetPids::Response> res)
{
    verbose_ = get_node()->get_parameter("verbose").as_bool();
    res->ack = true;

    for (std::size_t i = 0; i < req->data.size(); i++)
    {
        for (std::size_t j = 0; j < joint_names_.size(); j++)
        {
            if (!std::strcmp(joint_names_[j].c_str(), req->data[i].joint_name.c_str()))
            {
                if (req->data[i].p_value >= 0.0)
                {
                    joint_p_gain_[j] = req->data[i].p_value;
                    if (verbose_)
                        RCLCPP_INFO(get_node()->get_logger(), "Set P gain for joint %s to: %f",
                                    joint_names_[j].c_str(), joint_p_gain_[j]);
                }
                else { RCLCPP_WARN(get_node()->get_logger(), "P value has to be positive"); res->ack = false; }

                if (req->data[i].i_value >= 0.0)
                {
                    joint_i_gain_[j] = req->data[i].i_value;
                    if (verbose_)
                        RCLCPP_INFO(get_node()->get_logger(), "Set I gain for joint %s to: %f",
                                    joint_names_[j].c_str(), joint_i_gain_[j]);
                }
                else { RCLCPP_WARN(get_node()->get_logger(), "I value has to be positive"); res->ack = false; }

                if (req->data[i].d_value >= 0.0)
                {
                    joint_d_gain_[j] = req->data[i].d_value;
                    if (verbose_)
                        RCLCPP_INFO(get_node()->get_logger(), "Set D gain for joint %s to: %f",
                                    joint_names_[j].c_str(), joint_d_gain_[j]);
                }
                else { RCLCPP_WARN(get_node()->get_logger(), "D value has to be positive"); res->ack = false; }

                if (joint_i_gain_[j] == 0)
                {
                    use_integral_action_[j] = false;
                    T_i[j] = 0;
                }
                else
                {
                    use_integral_action_[j] = true;
                    T_i[j] = joint_p_gain_[j] / joint_i_gain_[j];
                }
                T_d[j] = (joint_p_gain_[j] == 0) ? 0 : joint_d_gain_[j] / joint_p_gain_[j];
            }
        }
    }
}

void Controller::commandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    const std::size_t n = joint_names_.size();
    if (n == msg->position.size() && n == msg->velocity.size() && n == msg->effort.size())
    {
        Command cmd;
        cmd.des_joint_positions = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), n);
        cmd.des_joint_velocities = Eigen::Map<const Eigen::VectorXd>(msg->velocity.data(), n);
        cmd.des_joint_efforts = Eigen::Map<const Eigen::VectorXd>(msg->effort.data(), n);
        command_buffer_.writeFromNonRT(cmd);
    }
    else
    {
        RCLCPP_WARN(get_node()->get_logger(), "Wrong dimension!");
    }
}

controller_interface::return_type Controller::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    const std::size_t n = joint_names_.size();

    // Fetch the latest command (real-time safe)
    Command * cmd = command_buffer_.readFromRT();
    if (cmd && cmd->des_joint_positions.size() == static_cast<Eigen::Index>(n))
    {
        des_joint_positions_ = cmd->des_joint_positions;
        des_joint_velocities_ = cmd->des_joint_velocities;
        des_joint_efforts_ = cmd->des_joint_efforts;
    }

    if (discrete_implementation_)
    {
        Ts = period.seconds();
        for (std::size_t i = 0; i < n; i++)
        {
            const double pos = state_interfaces_[2 * i].get_value();      // position
            error1_[i] = error_[i];
            error_[i] = des_joint_positions_(i) - pos;

            proportional_action_[i] = joint_p_gain_[i] * error_[i];
            if (use_integral_action_[i])
                integral_action_[i] += (joint_p_gain_[i] * Ts / T_i[i]) * error_[i];
            derivative_action_[i] = 1 / (1 + T_d[i] / (N * Ts)) *
                (T_d[i] / (N * Ts) * derivative_action_[i] +
                 joint_p_gain_[i] * T_d[i] / Ts * (error_[i] - error1_[i]));

            des_joint_efforts_pids_(i) = proportional_action_[i] + integral_action_[i] + derivative_action_[i];
            command_interfaces_[i].set_value(des_joint_efforts_(i) + des_joint_efforts_pids_(i));
            joint_positions_old_[i] = pos;
        }
    }
    else
    {
        for (std::size_t i = 0; i < n; i++)
        {
            measured_joint_position_(i) = state_interfaces_[2 * i].get_value();     // position
            measured_joint_velocity_(i) = state_interfaces_[2 * i + 1].get_value(); // velocity
            const double joint_pos_error = des_joint_positions_(i) - measured_joint_position_(i);
            const double integral_action = integral_action_old_[i] +
                joint_i_gain_[i] * joint_pos_error * period.seconds();
            des_joint_efforts_pids_(i) =
                joint_p_gain_[i] * (des_joint_positions_(i) - measured_joint_position_(i)) +
                joint_d_gain_[i] * (des_joint_velocities_(i) - measured_joint_velocity_(i)) +
                integral_action;
            integral_action_old_[i] = integral_action;
            command_interfaces_[i].set_value(des_joint_efforts_(i) + des_joint_efforts_pids_(i));
        }
    }

    // Publish the effort PID debug message (real-time safe)
    if (effort_pid_pub_ && effort_pid_pub_->trylock())
    {
        auto & msg = effort_pid_pub_->msg_;
        msg.header.stamp = get_node()->now();
        msg.name.resize(n);
        msg.effort_pid.resize(n);
        for (std::size_t i = 0; i < n; i++)
        {
            msg.name[i] = joint_names_[i];
            msg.effort_pid[i] = des_joint_efforts_pids_(i);
        }
        effort_pid_pub_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

} // namespace ros_impedance_controller

PLUGINLIB_EXPORT_CLASS(ros_impedance_controller::Controller,
                       controller_interface::ControllerInterface)
