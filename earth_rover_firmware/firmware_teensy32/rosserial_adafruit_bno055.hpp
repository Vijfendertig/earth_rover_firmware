#ifndef __ROSSERIAL_ADAFRUIT_BNO055__
#define __ROSSERIAL_ADAFRUIT_BNO055__


#include <EEPROM.h>

#include "adafruit_bno055.hpp"

#include <ros.h>
#include <std_msgs/Bool.h>
#include <earth_rover_firmware_msgs/Bno055Measurements.h>
#include <earth_rover_firmware_msgs/Bno055CalibrationStatus.h>


namespace earth_rover_firmware {

  using namespace earth_rover_firmware_msgs;
  

  template<typename I2CDevice>
  class RosserialAdafruitBNO055 {
    private:  // Data types and member variables.
      ros::NodeHandle * node_handle_;
      AdafruitBNO055<I2CDevice> sensor_;
      Bno055Measurements measurements_message_;
      Bno055CalibrationStatus calibration_status_message_;
      ros::Subscriber<std_msgs::Bool, RosserialAdafruitBNO055> enable_subscriber_;
      ros::Publisher measurements_publisher_;
      ros::Publisher calibration_status_publisher_;
      bool enable_;
      unsigned long int measurements_publish_interval_;
      unsigned long int measurements_last_published_;
      unsigned long int calibration_status_publish_interval_;
      unsigned long int calibration_status_last_published_;
      struct StoredCalibrationData {
        // Add a valid field before and after the calibration data to detect interrupted writes.
        uint8_t signature_front;
        typename AdafruitBNO055<I2CDevice>::bno055_calibration_values_t data;
        ros::Time timestamp;
        uint8_t signature_rear;
      };
      uint16_t calibration_slots_address_;
      uint8_t calibration_slots_count_;
      static constexpr uint8_t calibration_signature_ = 55U;
      uint8_t current_calibration_slot_;
    public:  // Member functions.
      RosserialAdafruitBNO055(ros::NodeHandle * node_handle, I2CDevice * i2c_device, uint8_t i2c_bno055_address,
          unsigned long int measurements_publish_interval, unsigned long int calibration_status_publish_interval,
          uint16_t calibration_slots_address = 0u, uint8_t calibration_slots_count = 8u);
      ~RosserialAdafruitBNO055() = default;
      uint16_t getEepromBaseAddress() const { return calibration_slots_address_; };
      uint16_t getEepromUsed() const { return uint16_t(calibration_slots_count_ * sizeof(StoredCalibrationData)); };
      void setup();
      void enable();
      void disable();
      void spinOnce();
    private:  // Member functions.
      void enableCallback(const std_msgs::Bool & message);
      void getAndPublishMeasurements();
      void getAndPublishCalibrationStatus();
      void resetStoredCalibrationData(StoredCalibrationData & data);
      void loadCalibrationFromEeprom();
      void saveCalibrationToEeprom();
  };


  template<typename I2CDevice>
  RosserialAdafruitBNO055<I2CDevice>::RosserialAdafruitBNO055(ros::NodeHandle * node_handle,
      I2CDevice * i2c_device, uint8_t i2c_bno055_address,
      unsigned long int measurements_publish_interval, unsigned long int calibration_status_publish_interval,
      uint16_t calibration_slots_address, uint8_t calibration_slots_count):
    node_handle_{node_handle},
    sensor_{i2c_device, i2c_bno055_address},
    enable_subscriber_{"bno055/enable", &RosserialAdafruitBNO055::enableCallback, this},
    measurements_publisher_{"bno055/imu", &measurements_message_},
    calibration_status_publisher_{"bno055/calib_status", &calibration_status_message_},
    enable_{false},
    measurements_publish_interval_{measurements_publish_interval},
    measurements_last_published_{0UL},
    calibration_status_publish_interval_{calibration_status_publish_interval},
    calibration_status_last_published_{0UL},
    calibration_slots_address_{calibration_slots_address},
    calibration_slots_count_{calibration_slots_count},
    current_calibration_slot_{uint8_t(calibration_slots_count_ - 1)}
  {
    measurements_message_.header.frame_id = "bno055_link";
    measurements_message_.header.seq = 0;
    calibration_status_message_.last_saved = ros::Time();
  }


  template<typename I2CDevice>
  void RosserialAdafruitBNO055<I2CDevice>::setup() {
    if(sensor_.setup()) {
      loadCalibrationFromEeprom();
      node_handle_->advertise(measurements_publisher_);
      node_handle_->advertise(calibration_status_publisher_);
      node_handle_->subscribe(enable_subscriber_);
    }
    else {
      pinMode(LED_BUILTIN, OUTPUT);
      while(true) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
      }
    }
  }

  template<typename I2CDevice>
  void RosserialAdafruitBNO055<I2CDevice>::enable() {
    if(enable_ == false) {
      measurements_last_published_ = millis() - measurements_publish_interval_;
      calibration_status_last_published_ = millis() - calibration_status_publish_interval_;
    }
    enable_ = true;
  }


  template<typename I2CDevice>
  void RosserialAdafruitBNO055<I2CDevice>::disable() {
    if(enable_ == true) {
      saveCalibrationToEeprom();
    }
    enable_ = false;
  }


  template<typename I2CDevice>
  void RosserialAdafruitBNO055<I2CDevice>::spinOnce() {
    if(enable_) {
      auto current = millis();
      if(current >= measurements_last_published_ + measurements_publish_interval_
          || current < measurements_last_published_) {
        getAndPublishMeasurements();
        measurements_last_published_ += measurements_publish_interval_;    
      }
      if(current >= calibration_status_last_published_ + calibration_status_publish_interval_
          || current < calibration_status_last_published_) {
        digitalWrite(LED_BUILTIN, HIGH);
        getAndPublishCalibrationStatus();
        digitalWrite(LED_BUILTIN, LOW);
        calibration_status_last_published_ += calibration_status_publish_interval_;    
      }
    }
  }


  template<typename I2CDevice>
  void RosserialAdafruitBNO055<I2CDevice>::enableCallback (const std_msgs::Bool & message) {
    if(message.data == true) {
      enable();
    }
    else {
      disable();
    }
  }


  template<typename I2CDevice>
  void RosserialAdafruitBNO055<I2CDevice>::getAndPublishMeasurements() {
    // Store timestamp in the message's header.
    measurements_message_.header.stamp = node_handle_->now();
    // Get measurements.
    auto quaternion = sensor_.getQuaternion();
    auto angular_velocity = sensor_.getAngularVelocity();
    auto linear_acceleration = sensor_.getLinearAcceleration();
    // Store absolute orientation (as a quaternion).
    measurements_message_.orientation.x = quaternion.x;
    measurements_message_.orientation.y = quaternion.y;
    measurements_message_.orientation.z = quaternion.z;
    measurements_message_.orientation.w = quaternion.w;
    // Store ansular velocity (in rad/s);
    measurements_message_.angular_velocity.x = angular_velocity.x;
    measurements_message_.angular_velocity.y = angular_velocity.y;
    measurements_message_.angular_velocity.z = angular_velocity.z;
    // Store linear acceleration (in m/s^2).
    measurements_message_.linear_acceleration.x = linear_acceleration.x;
    measurements_message_.linear_acceleration.y = linear_acceleration.y;
    measurements_message_.linear_acceleration.z = linear_acceleration.z;
    // Publish message.
    measurements_publisher_.publish(&measurements_message_);
    // Augment message sequence id.
    ++ measurements_message_.header.seq;
  }


  template<typename I2CDevice>
  void RosserialAdafruitBNO055<I2CDevice>::getAndPublishCalibrationStatus() {
    // Get calibration status.
    auto calibration_status = sensor_.getCalibrationStatus();
    calibration_status_message_.system = calibration_status.system;
    calibration_status_message_.accelerometer = calibration_status.accelerometer;
    calibration_status_message_.gyroscope = calibration_status.gyroscope;
    calibration_status_message_.magnetometer = calibration_status.magnetometer;
    // Publish message.
    calibration_status_publisher_.publish(&calibration_status_message_);
  }


  template<typename I2CDevice>
  void RosserialAdafruitBNO055<I2CDevice>::resetStoredCalibrationData(StoredCalibrationData & data) {
    data.signature_front = 0xff;
    data.data.accel_offset_x = 0xffff;
    data.data.accel_offset_y = 0xffff;
    data.data.accel_offset_z = 0xffff;
    data.data.mag_offset_x = 0xffff;
    data.data.mag_offset_y = 0xffff;
    data.data.mag_offset_z = 0xffff;
    data.data.gyro_offset_x = 0xffff;
    data.data.gyro_offset_y = 0xffff;
    data.data.gyro_offset_z = 0xffff;
    data.data.accel_radius = 0xffff;
    data.data.mag_radius = 0xffff;
    data.timestamp = ros::Time(0xffffffff, 0xffffffff);
    data.signature_rear = 0xff;
  }


  template<typename I2CDevice>
  void RosserialAdafruitBNO055<I2CDevice>::loadCalibrationFromEeprom() {
    int8_t preferred_calibration_slot{-1};
    ros::Time preferred_calibration_timestamp;
    StoredCalibrationData stored_calibration;
    for(int8_t calibration_slot = 0; calibration_slot < calibration_slots_count_; ++ calibration_slot) {
      EEPROM.get(calibration_slots_address_ + calibration_slot * sizeof(StoredCalibrationData), stored_calibration);
      if(stored_calibration.signature_front == calibration_signature_
         && stored_calibration.signature_rear == calibration_signature_
         && (stored_calibration.timestamp.sec > preferred_calibration_timestamp.sec
             || (stored_calibration.timestamp.sec == preferred_calibration_timestamp.sec
                 && stored_calibration.timestamp.nsec > preferred_calibration_timestamp.nsec))) {
        preferred_calibration_slot = calibration_slot;
        preferred_calibration_timestamp = stored_calibration.timestamp;
      }
    }
    if(preferred_calibration_slot != -1) {
      current_calibration_slot_ = preferred_calibration_slot;
      EEPROM.get(calibration_slots_address_ + preferred_calibration_slot * sizeof(StoredCalibrationData),
                 stored_calibration);
      sensor_.setCalibrationValues(stored_calibration.data);
      calibration_status_message_.last_saved = stored_calibration.timestamp;
    }
  }


  template<typename I2CDevice>
  void RosserialAdafruitBNO055<I2CDevice>::saveCalibrationToEeprom() {
    if(sensor_.isFullyCalibrated()) {
      // Get new calibration slot.
      uint8_t new_calibration_slot = (current_calibration_slot_ + 1) % calibration_slots_count_;
      // Create empty calibration message and write it to the calibration slot.
      StoredCalibrationData new_calibration;
      resetStoredCalibrationData(new_calibration);
      EEPROM.put(calibration_slots_address_ + new_calibration_slot * sizeof(StoredCalibrationData), new_calibration);
      // Get calibration from sensor and write it to the calibration slot.
      sensor_.getCalibrationValues(new_calibration.data);
      new_calibration.timestamp = node_handle_->now();
      new_calibration.signature_front = calibration_signature_;
      new_calibration.signature_rear = calibration_signature_;
      EEPROM.put(calibration_slots_address_ + new_calibration_slot * sizeof(StoredCalibrationData), new_calibration);
      current_calibration_slot_ = new_calibration_slot;
      calibration_status_message_.last_saved = new_calibration.timestamp;
    }
  }

}


#endif
