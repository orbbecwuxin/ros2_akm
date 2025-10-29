// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include <cmath>
#include "src/ord_lidar_driver.h"
#include <sys/time.h>

using namespace std;
using namespace ordlidar;

#define Degree2Rad(X) ((X)*M_PI / 180.)

void publish_msg(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr &pub, 
                 full_scan_data_st *scan_frame, 
                 rclcpp::Time start,
                 double scan_time, 
                 std::string frame_id, 
                 bool clockwise,
                 double angle_min, 
                 double angle_max, 
                 double min_range, 
                 double max_range)
{
  sensor_msgs::msg::LaserScan scanMsg;
  int point_nums = scan_frame->vailtidy_point_num;
  RCLCPP_INFO(rclcpp::get_logger("oradar_scan"), "scan point nums: %d", point_nums);
  scanMsg.header.stamp = start;
  scanMsg.header.frame_id = frame_id;
  scanMsg.angle_min = Degree2Rad(scan_frame->data[0].angle);
  scanMsg.angle_max = Degree2Rad(scan_frame->data[point_nums - 1].angle);
  double diff = scan_frame->data[point_nums - 1].angle - scan_frame->data[0].angle;
  RCLCPP_INFO(rclcpp::get_logger("oradar_scan"), "angle_min: %f, angle_max: %f, diff: %f",
               scan_frame->data[0].angle, scan_frame->data[point_nums - 1].angle, diff);
  scanMsg.angle_increment = Degree2Rad(diff/point_nums);
  scanMsg.scan_time = scan_time;
  scanMsg.time_increment = scan_time / point_nums;
  scanMsg.range_min = min_range;
  scanMsg.range_max = max_range;

  scanMsg.ranges.assign(point_nums, std::numeric_limits<float>::quiet_NaN());
  scanMsg.intensities.assign(point_nums, std::numeric_limits<float>::quiet_NaN());

  float range = 0.0;
  float intensity = 0.0;
  float dir_angle;
  unsigned int last_index = 0;

  for (int i = 0; i < point_nums; i++)
  {
    range = scan_frame->data[i].distance * 0.001;
    intensity = scan_frame->data[i].intensity;

    // if ((range > max_range) || (range < min_range))
    // {
    //   range = 0.0;
    //   intensity = 0.0;
    // }

    if (!clockwise)
    {
      dir_angle = static_cast<float>(360.f - scan_frame->data[i].angle);
    }
    else
    {
      dir_angle = scan_frame->data[i].angle;
    }

    if (dir_angle < angle_min)
    {
      range = -std::numeric_limits<float>::infinity();
      intensity = 0;
    }
    else if (dir_angle > angle_max)
    {
      range = std::numeric_limits<float>::infinity();
      intensity = 0;
      /* code */
    }
    
    if (range > max_range)
    {
      range = std::numeric_limits<float>::infinity();
      intensity = 0.0;
    }
    else if (range < min_range)
    {
      range = -std::numeric_limits<float>::infinity(); //无穷小
      intensity = 0.0;
    }


    float angle = Degree2Rad(dir_angle);
    unsigned int index = (unsigned int)((angle - scanMsg.angle_min) / scanMsg.angle_increment);
    if (index < point_nums)
    {
      // If the current content is Nan, it is assigned directly
      if (std::isnan(scanMsg.ranges[index]))
      {
        scanMsg.ranges[index] = range;
        unsigned int err = index - last_index;
        if (err == 2)
        {
          scanMsg.ranges[index - 1] = range;
          scanMsg.intensities[index - 1] = intensity;
        }
      }
      else
      { 
        // Otherwise, only when the distance is less than the current
        // value, it can be re assigned
        if (range < scanMsg.ranges[index])
        {
          scanMsg.ranges[index] = range;
        }
      }
      scanMsg.intensities[index] = intensity;
      last_index = index;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("oradar_scan"), "Publishing scan data");
  RCLCPP_INFO(rclcpp::get_logger("oradar_scan"), "angle_min: %f, angle_max: %f, angle_increment: %f",
               scanMsg.angle_min, scanMsg.angle_max, scanMsg.angle_increment);
  RCLCPP_INFO(rclcpp::get_logger("oradar_scan"), "range data.size: %d", scanMsg.ranges.size());
  RCLCPP_INFO(rclcpp::get_logger("oradar_scan"), "intensity data.size: %ld", scanMsg.intensities.size());
  RCLCPP_INFO(rclcpp::get_logger("oradar_scan"), "scan time: %f, time increment: %f",
               scanMsg.scan_time, scanMsg.time_increment);

  pub->publish(scanMsg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("oradar_scan");

  // Declare parameters with default values
  node->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
  node->declare_parameter<int>("baudrate", 230400);
  node->declare_parameter<double>("angle_max", 360.0);
  node->declare_parameter<double>("angle_min", 0.0);
  node->declare_parameter<double>("range_max", 20.0);
  node->declare_parameter<double>("range_min", 0.05);
  node->declare_parameter<bool>("clockwise", false);
  node->declare_parameter<int>("motor_speed", 10);
  node->declare_parameter<std::string>("device_model", "MS200");
  node->declare_parameter<std::string>("frame_id", "laser_frame");
  node->declare_parameter<std::string>("scan_topic", "scan");

  // Get parameters
  std::string port = node->get_parameter("port_name").as_string();
  int baudrate = node->get_parameter("baudrate").as_int();
  double angle_max = node->get_parameter("angle_max").as_double();
  double angle_min = node->get_parameter("angle_min").as_double();
  double max_range = node->get_parameter("range_max").as_double();
  double min_range = node->get_parameter("range_min").as_double();
  bool clockwise = node->get_parameter("clockwise").as_bool();
  int motor_speed = node->get_parameter("motor_speed").as_int();
  std::string device_model = node->get_parameter("device_model").as_string();
  std::string frame_id = node->get_parameter("frame_id").as_string();
  std::string scan_topic = node->get_parameter("scan_topic").as_string();

  // Create publisher
  auto publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 10);

  // Initialize lidar driver
  uint8_t type = ORADAR_TYPE_SERIAL;
  int model = ORADAR_MS200;
  OrdlidarDriver device(type, model);

  if (port.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Can't find lidar MS200");
    rclcpp::shutdown();
    return -1;
  }

  device.SetSerialPort(port, baudrate);

  RCLCPP_INFO(node->get_logger(), "Lidar type: %s", device_model.c_str());
  RCLCPP_INFO(node->get_logger(), "Serial port: %s, baudrate: %d", port.c_str(), baudrate);

  // Connect to lidar
  while (rclcpp::ok())
  {
    if (device.isConnected())
    {
      device.Disconnect();
      RCLCPP_INFO(node->get_logger(), "Disconnect lidar device");
    }

    if (device.Connect())
    {
      RCLCPP_INFO(node->get_logger(), "Lidar device connected successfully");
      break;
    }
    else
    {
      RCLCPP_INFO(node->get_logger(), "Connecting to lidar device...");
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }

  // Set motor speed
  double min_thr = (double)motor_speed - ((double)motor_speed * 0.1);
  double max_thr = (double)motor_speed + ((double)motor_speed * 0.1);
  double cur_speed = device.GetRotationSpeed();
  
  if (cur_speed < min_thr || cur_speed > max_thr)
  {
    device.SetRotationSpeed(motor_speed);
  }

  RCLCPP_INFO(node->get_logger(), "Getting lidar scan data");
  RCLCPP_INFO(node->get_logger(), "ROS2 topic: %s", scan_topic.c_str());

  full_scan_data_st scan_data;
  
  // Main loop
  while (rclcpp::ok())
  {
    rclcpp::Time start_scan_time = node->now();
    bool ret = device.GrabFullScanBlocking(scan_data, 1000);
    rclcpp::Time end_scan_time = node->now();
    
    double scan_duration = (end_scan_time - start_scan_time).seconds();

    if (ret)
    {
      publish_msg(publisher, &scan_data, start_scan_time, scan_duration, frame_id,
                  clockwise, angle_min, angle_max, min_range, max_range);
    }
    
    // Spin once to handle callbacks
    rclcpp::spin_some(node);
  }

  device.Disconnect();
  RCLCPP_INFO(node->get_logger(), "Node shutting down");
  
  rclcpp::shutdown();
  return 0;
}