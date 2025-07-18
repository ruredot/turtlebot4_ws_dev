/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Your Name
 */

 #ifndef IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__IMU_HPP_
 #define IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__IMU_HPP_
 
 #include <memory>
 #include <string>
 
 #include "rclcpp/rclcpp.hpp"
 #include "sensor_msgs/msg/imu.hpp"
 #include <sensor_msgs/msg/magnetic_field.hpp>
 
 namespace irobot_create_ignition_toolbox
 {
 
 class Imu
 {
 public:
   explicit Imu(std::shared_ptr<rclcpp::Node> & nh);
 
 private:
   std::shared_ptr<rclcpp::Node> nh_;
   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
   rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
 
   void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  // void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);

  // // 추가: 자기장 데이터 처리를 위한 변수
  // rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  // rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  // bool has_mag_data_ = false;
 };
 
 }  // namespace irobot_create_ignition_toolbox
 
 #endif  // IROBOT_CREATE_IGNITION_TOOLBOX__SENSORS__IMU_HPP_