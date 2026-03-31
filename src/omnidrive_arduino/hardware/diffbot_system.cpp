// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "omnidrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace omnidrive_arduino
{
hardware_interface::CallbackReturn OmniDriveArduinoHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  device = info_.hardware_parameters["device"];
  timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
  pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
  pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
  pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  rads_per_count = 2 * M_PI / enc_counts_per_rev;
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
  if(serial_comm_.IsOpen())
  {
    serial_comm_.Close();
  }
  serial_comm_.Open(device);
  serial_comm_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  for (int i = 0; i < 3; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", 3.0 - i);
  }

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
   
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  
  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  
  if(serial_comm_.IsOpen())
  {
    serial_comm_.Close();
  }
  
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OmniDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  if(!serial_comm_.IsOpen())
  {
    return hardware_interface::return_type::ERROR;
  }
  serial_comm_.FlushIOBuffers();
  serial_comm_.Write("e\r");

  std::string response="";
  try
  {
    serial_comm_.ReadLine(response, '\n', timeout_ms);
  }
  catch(const LibSerial::ReadTimeout&)
  {
    std::cerr << "ReadByte call has timed out" << std::endl;
  }
  // std::cout << response << std::endl;
  
  std::string delimiter = " ";
  size_t start = 0;
  size_t end = response.find(delimiter);
  
  int token_count = 0;
  std::string tokens;
  int val[3];
  while (end != std::string::npos && token_count < 3)
  {
    tokens = response.substr(start, end-start);
    start = end+delimiter.length();
    end = response.find(delimiter, start);
    val[token_count] = std::atoi(tokens.c_str());
    token_count++;
    
  }
  // std::cout << "Encoder Reading : " << val[0] << " " << val[1] << " " << val[2] << std::endl;
  
  double delta_seconds = period.seconds();

  double pos_prev = left_wheel_pos;
  left_wheel_pos = val[0] * rads_per_count;
  left_wheel_vel = (left_wheel_pos - pos_prev) / delta_seconds;
  
  pos_prev = right_wheel_pos;
  right_wheel_pos = val[1] * rads_per_count;
  right_wheel_vel = (right_wheel_pos - pos_prev) / delta_seconds;

  pos_prev = back_wheel_pos;
  back_wheel_pos = val[2] * rads_per_count;
  back_wheel_vel = (back_wheel_pos - pos_prev) / delta_seconds;
  
  // std::cout << "State Interface (Position) : " << left_wheel_pos << " " << right_wheel_pos << " " << back_wheel_pos << std::endl;
  // std::cout << "State Interface (Velocity) : " << left_wheel_vel << " " << right_wheel_vel << " " << back_wheel_vel << std::endl;
  
  set_state("left_wheel_joint/position",left_wheel_pos);
  set_state("right_wheel_joint/position",right_wheel_pos);
  set_state("back_wheel_joint/position",back_wheel_pos);


  set_state("left_wheel_joint/velocity",left_wheel_vel);
  set_state("right_wheel_joint/velocity",right_wheel_vel);
  set_state("back_wheel_joint/velocity",back_wheel_vel);

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type omnidrive_arduino ::OmniDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  int left_motor_vel_pwm = int(get_command("left_wheel_joint/velocity")/rads_per_count/loop_rate);
  int right_motor_vel_pwm = int(get_command("right_wheel_joint/velocity")/rads_per_count/loop_rate);
  int back_motor_vel_pwm = int(get_command("back_wheel_joint/velocity")/rads_per_count/loop_rate);
  
  // std::cout << "Command Interface (Velocity) : " << left_motor_vel_pwm << " " << right_motor_vel_pwm << " " << back_motor_vel_pwm << std::endl;

  std::stringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "m " << left_motor_vel_pwm << " " << right_motor_vel_pwm << " " << back_motor_vel_pwm << " " << "\r";
  
  if(!serial_comm_.IsOpen())
  {
    return hardware_interface::return_type::ERROR;
  }
  serial_comm_.FlushIOBuffers();
  serial_comm_.Write(ss.str());

  std::string response="";
  try
  {
    serial_comm_.ReadLine(response, '\n', timeout_ms);
  }
  catch(const LibSerial::ReadTimeout&)
  {
    std::cerr << "ReadByte call has timed out" << std::endl;
  }
  // std::cout << response << std::endl; 

  return hardware_interface::return_type::OK;
}

}  // namespace omnidrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  omnidrive_arduino::OmniDriveArduinoHardware, hardware_interface::SystemInterface)
  
