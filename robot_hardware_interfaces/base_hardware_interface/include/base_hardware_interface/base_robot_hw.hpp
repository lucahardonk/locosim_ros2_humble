/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

// =====================================================================================
// ROS2 (Humble) port of the WoLF base hardware interface.
//
// PORTING NOTES
// -------------
// In ROS1 (ros_control) this class derived its behaviour from the
// hardware_interface::RobotHW ecosystem and registered JointStateInterface,
// EffortJointInterface, ImuSensorInterface plus the custom GroundTruthInterface
// and ContactSwitchSensorInterface with a HardwareResourceManager. In ROS2
// (ros2_control) a real hardware plugin instead derives from
// hardware_interface::SystemInterface and exposes state/command through
// export_state_interfaces()/export_command_interfaces() as
// hardware_interface::StateInterface / CommandInterface objects, driven by the
// controller_manager lifecycle (on_init/on_configure/read/write).
//
// This translation keeps the *data model* (joint/imu/ground-truth/contact
// buffers) and the SRDF/URDF loading helpers, which are pure logic and are
// reused unchanged by both worlds. The old ros_control interface-registration
// calls are preserved as thin, self-contained helper registries (see
// ground_truth_interface.hpp / contact_switch_sensor_interface.hpp) so that a
// concrete SystemInterface subclass (e.g. for the Go1/AlienGo real robots) can
// build on top of it. The ROS1 ros::NodeHandle parameter lookups are replaced
// by an injected rclcpp node.
// =====================================================================================

#ifndef WOLF_ROBOT_HW_INTERFACE_HPP
#define WOLF_ROBOT_HW_INTERFACE_HPP

#include <deque>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <base_hardware_interface/ground_truth_interface.hpp>
#include <base_hardware_interface/contact_switch_sensor_interface.hpp>

// Forward declaration to avoid a hard dependency on srdfdom's headers in code
// that only needs the helper API.
namespace srdf { class Model; }

namespace hardware_interface
{

/// Minimal replacement for the ROS1 ImuSensorHandle data structure.
struct ImuSensorData
{
    std::string name;
    std::string frame_id;
    double* orientation{nullptr};
    double* angular_velocity{nullptr};
    double* linear_acceleration{nullptr};
};

class WolfRobotHwInterface
{
public:

    const std::string CLASS_NAME = "WolfRobotHwInterface";

    WolfRobotHwInterface();
    virtual ~WolfRobotHwInterface();

    void initializeJointsInterface(const std::vector<std::string>& joint_names);
    void initializeImuInterface(const std::string& imu_link_name);
    void initializeGroundTruthInterface(const std::string& base_link_name);
    void initializeContactSensorsInterface(const std::vector<std::string>& contact_names);

    std::string getRobotName() {return robot_name_;}
    unsigned int getNdof() {return n_dof_;}

    // The rclcpp node is used to read robot_description / robot_semantic_description
    // parameters (the ROS2 equivalent of the ROS1 global parameter server entries).
    void setNode(const rclcpp::Node::SharedPtr& node) {node_ = node;}

    std::vector<std::string> loadJointNamesFromSRDF();
    std::string loadImuLinkNameFromSRDF();
    std::string loadBaseLinkNameFromSRDF();
    std::vector<std::string> loadContactSensorNamesFromSRDF();

protected:

    rclcpp::Node::SharedPtr node_;

    std::string robot_name_;

    // Custom sensor registries (see ground_truth_interface.hpp / contact_switch_sensor_interface.hpp).
    GroundTruthInterface ground_truth_interface_;
    ContactSwitchSensorInterface contact_sensor_interface_;

    unsigned int n_dof_{0};
    std::vector<std::string> joint_names_;
    std::vector<std::string> contact_sensor_names_;
    std::vector<int> joint_types_;
    std::vector<double> joint_effort_limits_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_effort_command_;

    GroundTruthHandle::Data gt_data_;
    std::vector<double> base_orientation_;
    std::vector<double> base_ang_vel_;
    std::vector<double> base_ang_vel_prev_;
    std::vector<double> base_ang_acc_;
    std::vector<double> base_lin_acc_;
    std::vector<double> base_lin_pos_;
    std::vector<double> base_lin_vel_;
    std::vector<double> base_lin_vel_prev_;

    ImuSensorData imu_data_;
    std::vector<double> imu_orientation_;
    std::vector<double> imu_ang_vel_;
    std::vector<double> imu_lin_acc_;
    std::vector<double> imu_euler_;

    std::vector<std::string> leg_name_;
    std::vector<std::vector<double> > force_;
    std::vector<std::vector<double> > torque_;
    std::vector<std::vector<double> > normal_;
    std::deque<bool> contact_;


private:

    bool parseSRDF(srdf::Model& srdf_model);
};

}  // namespace hardware_interface

#endif  // WOLF_ROBOT_HW_INTERFACE_HPP
