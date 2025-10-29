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
  // Directly create the output message
  sensor_msgs::msg::LaserScan output_scan;
  output_scan.header.stamp = start;
  output_scan.header.frame_id = frame_id;
  output_scan.scan_time = scan_time;
  output_scan.range_min = min_range;
  output_scan.range_max = max_range;

  // --- Start: Create a temporary representation of the raw scan ---
  int point_nums = scan_frame->vailtidy_point_num;
  if (point_nums <= 1) {
    RCLCPP_WARN(rclcpp::get_logger("oradar_scan"), "Not enough points to publish scan.");
    return;
  }

  std::vector<float> raw_ranges(point_nums, std::numeric_limits<float>::infinity());
  std::vector<float> raw_intensities(point_nums, 0.0);
  float raw_angle_min, raw_angle_max, raw_angle_increment;

  float first_angle = static_cast<float>(360.f - scan_frame->data[0].angle);
  float last_angle = static_cast<float>(360.f - scan_frame->data[point_nums - 1].angle);

  if (first_angle < last_angle) {
    raw_angle_min = Degree2Rad(first_angle);
    raw_angle_max = Degree2Rad(last_angle);
  } else {
    raw_angle_min = Degree2Rad(last_angle);
    raw_angle_max = Degree2Rad(first_angle);
  }
  raw_angle_increment = (raw_angle_max - raw_angle_min) / (point_nums - 1);

  for (int i = 0; i < point_nums; i++) {
    float range = scan_frame->data[i].distance * 0.001;
    float intensity = scan_frame->data[i].intensity;
    float dir_angle = clockwise ? scan_frame->data[i].angle : 360.f - scan_frame->data[i].angle;

    if (range > max_range) range = std::numeric_limits<float>::infinity();
    else if (range < min_range) range = std::numeric_limits<float>::infinity();
    
    float angle = Degree2Rad(dir_angle);
    unsigned int index = (unsigned int)((angle - raw_angle_min) / raw_angle_increment);
    if (index < point_nums) {
      raw_ranges[index] = range;
      raw_intensities[index] = intensity;
    }
  }
  // --- End: Temporary representation ---

  // Interpolate to a fixed number of points for the output scan
  int target_points = 1800;
  output_scan.angle_min = Degree2Rad(angle_min);
  output_scan.angle_max = Degree2Rad(angle_max);
  output_scan.angle_increment = (output_scan.angle_max - output_scan.angle_min) / (target_points - 1);
  output_scan.time_increment = scan_time / (target_points - 1);
  
  output_scan.ranges.resize(target_points);
  output_scan.intensities.resize(target_points);

  for(int i = 0; i < target_points; i++) {
      float target_angle = output_scan.angle_min + i * output_scan.angle_increment;
      float float_idx = (target_angle - raw_angle_min) / raw_angle_increment;
      int idx_lower = static_cast<int>(floor(float_idx));
      int idx_upper = static_cast<int>(ceil(float_idx));

      if (idx_lower < 0) idx_lower = 0;
      if (idx_upper >= point_nums) idx_upper = point_nums - 1;

      if (idx_lower >= 0 && idx_upper < point_nums) {
          if (idx_lower == idx_upper) {
              output_scan.ranges[i] = raw_ranges[idx_lower];
              output_scan.intensities[i] = raw_intensities[idx_lower];
          } else {
              float weight_upper = float_idx - idx_lower;
              float weight_lower = 1.0f - weight_upper;
              output_scan.intensities[i] = weight_lower * raw_intensities[idx_lower] +
                                            weight_upper * raw_intensities[idx_upper];
              if (std::isinf(raw_ranges[idx_lower]) || std::isinf(raw_ranges[idx_upper])) {
                output_scan.ranges[i] = std::numeric_limits<float>::infinity();
                output_scan.intensities[i] = 0.0;
              } else if(abs(raw_ranges[idx_lower] - raw_ranges[idx_upper]) > 0.3) // If the difference is too large, consider it invalid
              {
                output_scan.ranges[i] = std::numeric_limits<float>::infinity();
                output_scan.intensities[i] = 0.0;

              } 
              else{
                output_scan.ranges[i] = weight_lower * raw_ranges[idx_lower] +
                                        weight_upper * raw_ranges[idx_upper];
              }


          }
      } else {
          output_scan.ranges[i] = std::numeric_limits<float>::infinity();
          output_scan.intensities[i] = 0.0;
      }
  }

  pub->publish(output_scan);
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