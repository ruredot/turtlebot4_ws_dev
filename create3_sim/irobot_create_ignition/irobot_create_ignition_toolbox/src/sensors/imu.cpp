/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Your Name
 */

 #include "irobot_create_ignition_toolbox/sensors/imu.hpp"

 #include <memory>
 #include <string>
//  #include <sensor_msgs/msg/magnetic_field.hpp>
 
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

    // // 추가: 자기장 데이터 발행
    // mag_pub_ = nh_->create_publisher<sensor_msgs::msg::MagneticField>(
    //   "imu/mag",
    //   rclcpp::SensorDataQoS());
    
    //   // 추가: 자기장 데이터 구독 (ignition에서 오는 데이터)
    // mag_sub_ = nh_->create_subscription<sensor_msgs::msg::MagneticField>(
    //   "_internal/imu/mag",
    //   rclcpp::SensorDataQoS(),
    //   std::bind(&Imu::mag_callback, this, std::placeholders::_1));
     
   RCLCPP_INFO(nh_->get_logger(), "IMU sensor initialized");
 }
 
 void Imu::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
 {
   // 필요한 경우 IMU 데이터 처리 (예: 좌표 변환, 필터링 등)
   // 여기에서는 단순히 메시지를 그대로 전달
   imu_pub_->publish(*msg);

  //    // 자기장 데이터가 없는 경우, 시뮬레이션된 자기장 데이터 생성
  // if (!has_mag_data_) {
  //   auto mag_msg = std::make_shared<sensor_msgs::msg::MagneticField>();
  //   mag_msg->header = msg->header;
  //   // 지구 자기장 강도의 근사값 (북반구 기준, 단위: Tesla)
  //   mag_msg->magnetic_field.x = 18.0e-6;
  //   mag_msg->magnetic_field.y = 0.0;
  //   mag_msg->magnetic_field.z = 43.0e-6;
  //   mag_pub_->publish(*mag_msg);
  // }
 }

//  // 추가: 자기장 데이터 콜백 함수
// void Imu::mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
// {
//   has_mag_data_ = true;
//   mag_pub_->publish(*msg);
// }