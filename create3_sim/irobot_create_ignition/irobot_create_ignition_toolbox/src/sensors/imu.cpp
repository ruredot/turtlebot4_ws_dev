/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Your Name
 */

 #include "irobot_create_ignition_toolbox/sensors/imu.hpp"

 #include <memory>
 #include <string>
 
 using irobot_create_ignition_toolbox::Imu;
 
 Imu::Imu(std::shared_ptr<rclcpp::Node> & nh)
 : nh_(nh)
 {
   // 구독: Ignition으로부터 IMU 메시지를 수신
   imu_sub_ = nh_->create_subscription<sensor_msgs::msg::Imu>(
     "_internal/imu",
     rclcpp::SensorDataQoS(),
     std::bind(&Imu::imu_callback, this, std::placeholders::_1));
 
   // 발행: 최종 처리된 IMU 메시지를 발행
   imu_pub_ = nh_->create_publisher<sensor_msgs::msg::Imu>(
     "imu",
     rclcpp::SensorDataQoS());
     
   RCLCPP_INFO(nh_->get_logger(), "IMU sensor initialized");
 }
 
 void Imu::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
 {
   // 필요한 경우 IMU 데이터 처리 (예: 좌표 변환, 필터링 등)
   // 여기에서는 단순히 메시지를 그대로 전달
   imu_pub_->publish(*msg);
 }