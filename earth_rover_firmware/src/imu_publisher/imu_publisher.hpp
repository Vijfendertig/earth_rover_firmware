//! Imu (re)publisher node for BNO055 measurements.
/*!
 *  This node republishes the compact (excluding covariances) Bno055Measurements message from the Teensy 3.2 Adafruit
 *  BNO055 rosserial node as standard sensor_msgs/Imu messages, taking into account the calibration status of the
 *  Adafruit BNO055 sensor.
 * 
 *  \file
 * 
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_FIRMWARE__IMU_PUBLISHER__
#define __EARTH_ROVER_FIRMWARE__IMU_PUBLISHER__


#include <ros/ros.h>
#include <earth_rover_firmware_msgs/Bno055CalibrationStatus.h>
#include <earth_rover_firmware_msgs/Bno055Measurements.h>


namespace earth_rover_firmware {

  class ImuPublisher {
    private:
      ros::NodeHandle node_handle_;
      ros::Subscriber subscriber_compact_imu_;
      ros::Subscriber subscriber_calibration_status_;
      ros::Publisher publisher_full_imu_;
      std::string frame_id_;
      earth_rover_firmware_msgs::Bno055CalibrationStatus cached_calibration_status_;
    public:
      ImuPublisher(const std::string & frame_id);
      ~ImuPublisher() = default;
      void compactImuCallback(const earth_rover_firmware_msgs::Bno055Measurements::ConstPtr & message);
      void calibrationStatusCallback(const earth_rover_firmware_msgs::Bno055CalibrationStatus::ConstPtr & message);
  };

}


#endif