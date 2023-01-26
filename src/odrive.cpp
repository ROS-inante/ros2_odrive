/*
* Copyright (C) 2021 Alexander Junk <dev@junk.technology>
* 
* This code is based on the ros_odrive repository by Ioannis Kokkoris
* (https://github.com/johnkok/ros_odrive), which is distributed under the
* MIT license.
* 
* This program is free software: you can redistribute it and/or modify it 
* under the terms of the GNU Lesser General Public License as published 
* by the Free Software Foundation, either version 3 of the License, or 
* (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. 
* See the GNU Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License 
* along with this program. If not, see <https://www.gnu.org/licenses/>. 
*
*/


#include "ros2_odrive/odrive.hpp"
#include "ros2_odrive/to_string.hpp"


int main(int argc, char * argv[])
{

  std::string od_sn;
  std::string od_cfg;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ODrive>());
 
 
  rclcpp::shutdown();
  return 0;
}


ODrive::ODrive()
      : Node("odrive"), endpoint_{NULL}
  {

    auto serial_number = this->declare_parameter<std::string>("serial_number", "");
    this->declare_parameter<std::string>("config_file", "");

    if (serial_number.empty())
      RCLCPP_ERROR(this->get_logger(), "No serial number provided. Can not initialize ODRIVE node!");
    else
    {
      RCLCPP_INFO(this->get_logger(), "SN: %s", serial_number.c_str());

      endpoint_ = new odrive_endpoint();

      if (endpoint_->init(serial_number))
      {
        RCLCPP_ERROR(this->get_logger(), "Could not open USB endpoin!");
        delete endpoint_;
        endpoint_ = nullptr;
      }
    }

    if(endpoint_ && endpoint_->getDescriptor()){
      RCLCPP_ERROR(this->get_logger(), "Failed to retreive JSON description from ODRIVE!");
    }else{
      RCLCPP_INFO(this->get_logger(), "Got endpoint desciption.");
      // Process configuration file
      float speed = 5.0;
      endpoint_->writeOdriveData(std::string("axis0.controller.input_vel"), speed);
      
      //ToDo: Handle configuration via ROS-Parameters
      //updateTargetConfig(endpoint_, json_, cfg_);
    }

    RCLCPP_INFO(this->get_logger(), "Creating publisher and subscriber.");

    status_publisher_ = this->create_publisher<ros2_odrive_interfaces::msg::Status>("odrive_status", 10);

    error_publishers_.push_back(
      this->create_publisher<ros2_odrive_interfaces::msg::Error>("axis0/error", 10)
      );

    error_publishers_.push_back(
      this->create_publisher<ros2_odrive_interfaces::msg::Error>("axis1/error", 10)
      );

    velocity_publishers_.push_back(
      this->create_publisher<ros2_odrive_interfaces::msg::Velocity>("axis0/vel_estimate", 10)
      );

    velocity_publishers_.push_back(
      this->create_publisher<ros2_odrive_interfaces::msg::Velocity>("axis1/vel_estimate", 10)
      );




    ctrl_subscriber_ = this->create_subscription<ros2_odrive_interfaces::msg::Ctrl>(
        "odrive_ctrl", 10, std::bind(&ODrive::ctrl_topic_callback, this, _1));

    velocity_subscribers_.push_back(
      this->create_subscription<ros2_odrive_interfaces::msg::Velocity>(
        "axis0/cmd_vel", 10, std::bind(&ODrive::cmd_vel_topic_callback<0>, this, _1))
      );

    velocity_subscribers_.push_back(
      this->create_subscription<ros2_odrive_interfaces::msg::Velocity>(
        "axis1/cmd_vel", 10, std::bind(&ODrive::cmd_vel_topic_callback<1>, this, _1))
      );

    timer_ = this->create_wall_timer(
        500ms, std::bind(&ODrive::timer_callback, this));
      

    RCLCPP_INFO(this->get_logger(), "ODrive node finished setup!");
  }

ODrive::~ODrive()
{
    endpoint_->remove();
    delete endpoint_;
}

/**
 *
 * Publish odrive_status_topic
 * return ODRIVE_OK on success
 *
 */
int ODrive::publish_odrive_status() const
{
    //uint16_t u16val;
    uint32_t u32val;
    float fval;
    auto msg = ros2_odrive_interfaces::msg::Status();

    // Collect data
    endpoint_->readOdriveData(std::string("vbus_voltage"), fval);
    msg.vbus = fval;
    endpoint_->readOdriveData(std::string("axis0.error"), u32val);
    msg.error0 = u32val;
    endpoint_->readOdriveData(std::string("axis1.error"), u32val);
    msg.error1 = u32val;
    endpoint_->readOdriveData(std::string("axis0.current_state"), u32val);
    msg.state0 = u32val;
    endpoint_->readOdriveData(std::string("axis1.current_state"), u32val);
    msg.state1 = u32val;
    endpoint_->readOdriveData(std::string("axis0.encoder.vel_estimate"), fval);
    msg.vel0 = fval;
    endpoint_->readOdriveData(std::string("axis1.encoder.vel_estimate"), fval);
    msg.vel1 = fval;
    endpoint_->readOdriveData(std::string("axis0.encoder.pos_estimate"), fval);
    msg.pos0 = fval;
    endpoint_->readOdriveData(std::string("axis1.encoder.pos_estimate"), fval);
    msg.pos1 = fval;
    endpoint_->readOdriveData(std::string("axis0.motor.current_meas_phB"), fval);
    msg.curr0b = fval;
    endpoint_->readOdriveData(std::string("axis0.motor.current_meas_phC"), fval);
    msg.curr0c = fval;
    endpoint_->readOdriveData(std::string("axis1.motor.current_meas_phB"), fval);
    msg.curr1b = fval;
    endpoint_->readOdriveData(std::string("axis1.motor.current_meas_phC"), fval);

    // Publish message
    this->status_publisher_->publish(msg);

    return ODRIVE_OK;
}

// ODrive interfaceing functions

template<unsigned int axis> int ODrive::cmd_set_vel(float vel) const
{
  const std::string cmd = std::string("axis") + std::string(to_string<axis>) + std::string(".controller.input_vel");
  return endpoint_->writeOdriveData(cmd , vel);
}

template<unsigned int axis> int ODrive::cmd_set_pos(float pos) const
{
  const std::string cmd = std::string("axis") + std::string(to_string<axis>) + std::string(".controller.input_pos");
  return endpoint_->writeOdriveData(cmd , pos);
}

template<unsigned int axis> int ODrive::cmd_set_state(AXIS_STATE state) const
{
  const std::string cmd = std::string("axis") + std::string(to_string<axis>) + std::string(".requested_state");
  return endpoint_->writeOdriveData(cmd , utils::underlying_value(state));
}

void ODrive::ctrl_topic_callback(const ros2_odrive_interfaces::msg::Ctrl::SharedPtr msg) const
{
    if (! ((msg->axis == 0) || (msg->axis == 1)) ) {
      RCLCPP_ERROR(this->get_logger(), "Received invalid axis value in ctrl message!");
      return;
    }

    //ToDo: remove target from messages, we only have one
   /*  if ((msg->target < 0) || (msg->target >= MAX_NR_OF_TARGETS)) {
        RCLCPP_ERROR(this->get_logger(), "Received invalid target value in ctrl message!");
        return;
    } */

    switch (msg->command) {
        case (ODRIVE_CMD::AXIS_RESET):
            // Reset errors
            endpoint_->execOdriveFunc("axis0.clear_errors");
            endpoint_->execOdriveFunc("axis1.clear_errors");

            break;

	case (ODRIVE_CMD::AXIS_IDLE):
            // Set channel to Idle
            if(!msg->axis)
              cmd_set_state<0>(AXIS_STATE::IDLE);
            else
              cmd_set_state<1>(AXIS_STATE::IDLE);
            
            break;

	case (ODRIVE_CMD::AXIS_CLOSED_LOOP):
            // Enable Closed Loop Control
            if(!msg->axis)
              cmd_set_state<0>(AXIS_STATE::CLOSED_LOOP_CONTROL);
            else
              cmd_set_state<1>(AXIS_STATE::CLOSED_LOOP_CONTROL);

            break;

        case (ODRIVE_CMD::AXIS_SET_VELOCITY):
            // Set velocity

            if(!msg->axis)
              cmd_set_vel<0>(msg->fval);
            else
              cmd_set_vel<1>(msg->fval);
            
            break;

	case (ODRIVE_CMD::AXIS_SET_VELOCITY_DUAL):
            // Set velocity

             

            break;

        case (ODRIVE_CMD::REBOOT):
            endpoint_->execOdriveFunc(std::string("reboot"));
            break;

        default:
            RCLCPP_ERROR(this->get_logger(), "Received invalid command type in ctrl message!");
            return;
    }
}

// SUBSCRIBER


template<unsigned int AXIS> void ODrive::cmd_vel_topic_callback(const ros2_odrive_interfaces::msg::Velocity::SharedPtr msg) const{
  cmd_set_vel<AXIS>(msg->rps);
}

template<unsigned int AXIS>  void ODrive::cmd_pos_topic_callback(const ros2_odrive_interfaces::msg::Position::SharedPtr msg) const{
  cmd_set_pos<AXIS>(msg->pos);
}


// PUBLISHER

template<unsigned int AXIS> int ODrive::publish_error(void) const{
  
  auto msg = ros2_odrive_interfaces::msg::Error();

  endpoint_->readOdriveData(std::string("axis") + std::to_string(AXIS) + std::string(".error"), msg.err);

  this->error_publishers_.at(AXIS)->publish(msg);

  return ODRIVE_OK;
}

template<unsigned int AXIS> int ODrive::publish_vel_estimate(void) const
{
  auto msg = ros2_odrive_interfaces::msg::Velocity();

  endpoint_->readOdriveData(std::string("axis") + std::to_string(AXIS) + std::string(".encoder.vel_estimate"), msg.rps);

  velocity_publishers_.at(AXIS)->publish(msg);

  return ODRIVE_OK;
}