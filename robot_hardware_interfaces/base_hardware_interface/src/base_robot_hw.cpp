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

// ROS2 (Humble) port. See base_robot_hw.hpp for the porting rationale.

#include <base_hardware_interface/base_robot_hw.hpp>

#include <stdexcept>

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <srdfdom/srdf_writer.h>

using namespace hardware_interface;

static rclcpp::Logger hw_logger()
{
  return rclcpp::get_logger("WolfRobotHwInterface");
}

WolfRobotHwInterface::WolfRobotHwInterface()
{
}

WolfRobotHwInterface::~WolfRobotHwInterface()
{
}

void WolfRobotHwInterface::initializeJointsInterface(const std::vector<std::string>& joint_names)
{
    // Resize vectors to our DOF
    n_dof_ = static_cast<unsigned int>(joint_names.size());
    joint_names_.resize(n_dof_);
    joint_types_.resize(n_dof_);
    joint_effort_limits_.resize(n_dof_);
    joint_position_.resize(n_dof_);
    joint_velocity_.resize(n_dof_);
    joint_effort_.resize(n_dof_);
    joint_effort_command_.resize(n_dof_);

    for (unsigned int j = 0; j < n_dof_; j++)
    {
        RCLCPP_DEBUG_STREAM(hw_logger(), "Loading joint: " << joint_names[j]);

        joint_names_[j]          = joint_names[j];
        joint_position_[j]       = 1.0;
        joint_velocity_[j]       = 0.0;
        joint_effort_[j]         = 0.0;  // N/m for continuous joints
        joint_effort_command_[j] = 0.0;

        // In ROS2 ros2_control these per-joint buffers are exported through
        // export_state_interfaces() (position/velocity/effort) and
        // export_command_interfaces() (effort) inside the SystemInterface
        // subclass, rather than being registered here with a JointStateInterface /
        // EffortJointInterface as in ROS1.
    }
}

void WolfRobotHwInterface::initializeImuInterface(const std::string& imu_link_name)
{
    imu_orientation_.resize(4);
    imu_ang_vel_.resize(3);
    imu_lin_acc_.resize(3);
    imu_euler_.resize(3);

    imu_euler_[0] = 0.0;
    imu_euler_[1] = 0.0;
    imu_euler_[2] = 0.0;

    imu_data_.name = "imu";
    imu_data_.frame_id = imu_link_name;
    imu_data_.orientation = &imu_orientation_[0];
    imu_data_.angular_velocity = &imu_ang_vel_[0];
    imu_data_.linear_acceleration = &imu_lin_acc_[0];
    // ROS2: the IMU state is exposed as StateInterfaces (imu/orientation.x, ...)
    // by the SystemInterface subclass.
}

void WolfRobotHwInterface::initializeGroundTruthInterface(const std::string& base_link_name)
{
    base_orientation_.resize(4);
    base_ang_vel_.resize(3);
    base_ang_vel_prev_.resize(3);
    base_ang_acc_.resize(3);
    base_lin_acc_.resize(3);
    base_lin_pos_.resize(3);
    base_lin_vel_.resize(3);
    base_lin_vel_prev_.resize(3);

    gt_data_.name = "ground_truth";
    gt_data_.frame_id = base_link_name;
    gt_data_.orientation = &base_orientation_[0];
    gt_data_.angular_velocity = &base_ang_vel_[0];
    gt_data_.angular_acceleration = &base_ang_acc_[0];
    gt_data_.linear_acceleration = &base_lin_acc_[0];
    gt_data_.linear_position = &base_lin_pos_[0];
    gt_data_.linear_velocity = &base_lin_vel_[0];
    ground_truth_interface_.registerHandle(hardware_interface::GroundTruthHandle(gt_data_));
}

void WolfRobotHwInterface::initializeContactSensorsInterface(const std::vector<std::string>& contact_names)
{
     for (unsigned int i = 0; i < contact_names.size(); i++)
        contact_sensor_names_.push_back(contact_names[i]);

     // Create the handle for each contact sensor.
     // Note: std::deque<bool> (unlike std::vector<bool>) stores real bool objects,
     // so &contact_[i] yields a valid bool* to hand to the sensor handle.
     contact_.resize(contact_sensor_names_.size());
     force_.resize(contact_sensor_names_.size());
     torque_.resize(contact_sensor_names_.size());
     normal_.resize(contact_sensor_names_.size());
     for (unsigned int i = 0; i < contact_sensor_names_.size(); i++)
     {
         contact_[i] = false;
         force_[i].resize(3, 0);
         torque_[i].resize(3, 0);
         normal_[i].resize(3, 0);
         contact_sensor_interface_.registerHandle(hardware_interface::ContactSwitchSensorHandle(
             contact_sensor_names_[i], &contact_[i], &force_[i][0], &torque_[i][0], &normal_[i][0]));
     }
}

std::vector<std::string> WolfRobotHwInterface::loadJointNamesFromSRDF()
{
    std::vector<std::string> joint_names;
    srdf::Model srdf_model;
    if (parseSRDF(srdf_model))
    {
        auto group_states = srdf_model.getGroupStates();
        for (unsigned int i = 0; i < group_states.size(); i++)
            if (group_states[i].name_ == "standup")  // Look for the standup group state and get the names of the joints in there
                for (auto & tmp : group_states[i].joint_values_)
                    joint_names.push_back(tmp.first);
    }
    return joint_names;
}

std::string WolfRobotHwInterface::loadImuLinkNameFromSRDF()
{
    std::string imu_name;
    srdf::Model srdf_model;

    if (parseSRDF(srdf_model))
    {
        auto groups = srdf_model.getGroups();
        for (unsigned int i = 0; i < groups.size(); i++)
        {
            const auto& links = groups[i].links_;
            if (groups[i].name_.find("imu") != std::string::npos)
            {
                if (links.size() == 1)
                    imu_name = links[0];
                else
                    throw std::runtime_error("There can be only one imu_sensor defined in the SRDF file!");
            }
        }
    }
    return imu_name;
}

std::string WolfRobotHwInterface::loadBaseLinkNameFromSRDF()
{
    std::string base_name;
    srdf::Model srdf_model;
    if (parseSRDF(srdf_model))
    {
        auto groups = srdf_model.getGroups();
        for (unsigned int i = 0; i < groups.size(); i++)
        {
            const auto& links = groups[i].links_;
            if (groups[i].name_.find("base") != std::string::npos)
            {
                if (links.size() == 1)
                    base_name = links[0];
                else
                    throw std::runtime_error("There can be only one base defined in the SRDF file!");
            }
        }
    }
    return base_name;
}

std::vector<std::string> WolfRobotHwInterface::loadContactSensorNamesFromSRDF()
{
    std::vector<std::string> contact_names;
    srdf::Model srdf_model;
    if (parseSRDF(srdf_model))
    {
        auto groups = srdf_model.getGroups();
        for (unsigned int i = 0; i < groups.size(); i++)
        {
            const auto& links = groups[i].links_;
            if (groups[i].name_.find("contact_sensors") != std::string::npos)
                for (unsigned int j = 0; j < links.size(); j++)
                    contact_names.push_back(links[j]);
        }
    }
    return contact_names;
}

bool WolfRobotHwInterface::parseSRDF(srdf::Model& srdf_model)
{
  if (!node_)
  {
      RCLCPP_ERROR(hw_logger(), "No rclcpp node set on WolfRobotHwInterface (call setNode() first).");
      return false;
  }

  // In ROS2 the URDF/SRDF are passed as node parameters (robot_description /
  // robot_semantic_description) instead of living on a global parameter server.
  std::string urdf, srdf;
  node_->get_parameter_or<std::string>("robot_description", urdf, std::string());
  node_->get_parameter_or<std::string>("robot_semantic_description", srdf, std::string());

  if (urdf.empty())
  {
      RCLCPP_ERROR(hw_logger(), "robot_description not available as a node parameter");
      return false;
  }
  if (srdf.empty())
  {
      RCLCPP_ERROR(hw_logger(), "robot_semantic_description not available as a node parameter");
      return false;
  }

  urdf::ModelInterfaceSharedPtr u = urdf::parseURDF(urdf);
  if (!srdf_model.initString(*u, srdf))
  {
      RCLCPP_ERROR(hw_logger(), "Can not initialize SRDF model from XML string!");
      return false;
  }

  return true;
}
