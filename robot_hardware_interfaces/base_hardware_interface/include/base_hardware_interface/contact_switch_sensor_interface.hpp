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

// ROS2 (Humble) port. See ground_truth_interface.hpp for the rationale: the
// ROS1 HardwareResourceManager base is replaced by a lightweight name->handle
// registry; in a full ros2_control SystemInterface the contact state would be
// exposed via StateInterface objects.

#ifndef HARDWARE_INTERFACE_CONTACT_SWITCH_SENSOR_INTERFACE_HPP
#define HARDWARE_INTERFACE_CONTACT_SWITCH_SENSOR_INTERFACE_HPP

#include <cstddef>
#include <map>
#include <string>
#include <stdexcept>

namespace hardware_interface
{

/// A handle used to read the state of a contact-switch sensor.
class ContactSwitchSensorHandle
{
public:
    ContactSwitchSensorHandle() : name_(""), contact_(nullptr), force_(nullptr), torque_(nullptr), normal_(nullptr) {}

    /**
     * \param name The name of the sensor
     * \param contact A pointer to the storage of the contact value
     */
    ContactSwitchSensorHandle(const std::string& name,
                              bool* contact,
                              double* force,
                              double* torque,
                              double* normal) :
                                  name_(name),
                                  contact_(contact),
                                  force_(force),
                                  torque_(torque),
                                  normal_(normal){}

    std::string getName() const {return name_;}
    const bool* getContactState() const {return contact_;}
    const double* getForce() const {return force_;}
    const double* getTorque() const {return torque_;}
    const double* getNormal() const {return normal_;}

private:
    std::string name_;
    bool* contact_;
    double* force_;
    double* torque_;
    double* normal_;
};

/** \brief Lightweight replacement for the ROS1 HardwareResourceManager<ContactSwitchSensorHandle>. */
class ContactSwitchSensorInterface
{
public:
  void registerHandle(const ContactSwitchSensorHandle& handle)
  {
    handles_[handle.getName()] = handle;
  }

  ContactSwitchSensorHandle getHandle(const std::string& name)
  {
    auto it = handles_.find(name);
    if (it == handles_.end())
      throw std::logic_error("Could not find contact-switch handle with name '" + name + "'.");
    return it->second;
  }

private:
  std::map<std::string, ContactSwitchSensorHandle> handles_;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE_CONTACT_SWITCH_SENSOR_INTERFACE_HPP
