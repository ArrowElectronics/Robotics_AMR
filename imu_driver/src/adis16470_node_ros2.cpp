// Copyright (c) 2017, Analog Devices Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

// ===================================================================
//
// Copyright (c) 2023 e-Infochips Private Limited
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:#
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// If any term or provision set forth herein is deemed to be invalid, illegal, or unenforceable in any jurisdiction, such invalidity, illegality, or unenforceability will not affect any other term or provision or invalidate or render unenforceable such term or provision in any other jurisdiction. Upon a court determination that any term or provision is invalid, illegal, or unenforceable, the court may modify these terms and conditions to affect our original intent as closely as possible in order that the transactions contemplated hereby be consummated to the greatest extent possible as originally contemplated.
// ===================================================================


#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "imu_driver/adis16470.h"
#include "std_srvs/srv/trigger.hpp"
#include <unistd.h>

class ImuNode : public rclcpp::Node
{
public:
  rclcpp::TimerBase::SharedPtr timer_;
  Adis16470 imu;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_data_pub_;
  std::string device_ {std::string("/dev/ttyACM0")};
  std::string frame_id_ {std::string("imu")};
  bool burst_mode_ {false};
  bool publish_temperature_ {true};
  ::std::chrono::milliseconds rate_ {100};

  ImuNode()
  : Node("ImuNode")
  {
    RCLCPP_INFO(this->get_logger(), "device: %s", device_.c_str());
    RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "rate: %lu [Hz]", rate_.count());
    RCLCPP_INFO(this->get_logger(), "burst_mode: %s", (burst_mode_ ? "true": "false"));
    RCLCPP_INFO(this->get_logger(), "publish_temperature: %s", (publish_temperature_ ? "true": "false"));

    timer_ = this->create_wall_timer(rate_, std::bind(&ImuNode::spin, this));

    // Data publisher
    imu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 100);
    if (publish_temperature_)
      {
        temp_data_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 100);
      }
  }

  ~ImuNode()
  {
    imu.closePort();
  }

  /**
   * @brief Check if the device is opened
   */
  bool is_opened(void)
  {
    return (imu.fd_ >= 0);
  }
  /**
   * @brief Open IMU device file
   */
  bool open(void)
  {
    // Open device file
    if (imu.openPort(device_) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open device %s", device_.c_str());
      return false;
    }
    // Wait 10ms for SPI ready
    usleep(10000);
    int16_t pid = 0;
    imu.get_product_id(pid);
    RCLCPP_INFO(this->get_logger(), "Product ID: %x\n", pid);
    imu.set_bias_estimation_time(0x070a);
    return true;
  }
  
  int publish_imu_data()
  {
    sensor_msgs::msg::Imu data;
    data.header.frame_id = frame_id_;
    data.header.stamp = this->get_clock()->now();

    // Linear acceleration
    data.linear_acceleration.x = imu.accl[0];
    data.linear_acceleration.y = imu.accl[1];
    data.linear_acceleration.z = imu.accl[2];

    // Angular velocity
    data.angular_velocity.x = imu.gyro[0];
    data.angular_velocity.y = imu.gyro[1];
    data.angular_velocity.z = imu.gyro[2];

    // Orientation (not provided)
    data.orientation.x = 0;
    data.orientation.y = 0;
    data.orientation.z = 0;
    data.orientation.w = 1;

    imu_data_pub_->publish(data);
    return 0;
  }
  int publish_temp_data()
  {
    sensor_msgs::msg::Temperature data;
    data.header.frame_id = frame_id_;
    data.header.stamp = this->get_clock()->now();

    // imu Temperature
    data.temperature = imu.temp;
    data.variance = 0;
    
    temp_data_pub_->publish(data);
    return 0;
  }
  void spin()
  {
      if (burst_mode_)
      {
        if (imu.update_burst() == 0)
        {
          publish_imu_data();
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Cannot update burst");
        }
      }
      else if (publish_temperature_)
      {
        if (imu.update() == 0)
        {
          publish_imu_data();
          publish_temp_data();
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Cannot update");
        }
      }
      else if (burst_mode_ && publish_temperature_)
      {
        if (imu.update_burst() == 0)
        {
          publish_imu_data();
          publish_temp_data();
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Cannot update burst");
        }
      }
      else
      {
        if (imu.update() == 0)
        {
          publish_imu_data();
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Cannot update");
        }
      }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuNode>();

  node->open();
  while (!node->is_opened())
  {
    RCLCPP_WARN(node->get_logger(), "Keep trying to open the device in 1 second period...");
    sleep(1);
    node->open();
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return(0);
}
