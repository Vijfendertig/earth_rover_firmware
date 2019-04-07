#ifndef __ROSSERIAL_POSITION_ENCODER__
#define __ROSSERIAL_POSITION_ENCODER__


#include <functional>
#include "isr_wrapper.hpp"
#include <ros.h>
#include <std_msgs/Bool.h>
#include <earth_rover_firmware_msgs/EncoderMeasurements.h>


namespace earth_rover_firmware {

  using namespace earth_rover_firmware_msgs;


  template <int hall_a_pin, int hall_b_pin>
  class RosserialPositionEncoder {
    private:
      ros::NodeHandle * node_handle_;
      ISRWrapper<digitalPinToInterrupt(hall_a_pin)> isr_wrapper_;
      volatile uint16_t measurement_position_;
      EncoderMeasurements measurements_message_;
      ros::Subscriber<std_msgs::Bool, RosserialPositionEncoder> enable_subscriber_;
      ros::Publisher measurements_publisher_;
      bool enable_;
      unsigned long int measurements_publish_interval_;
      unsigned long int measurements_last_published_;
    public:
      RosserialPositionEncoder(ros::NodeHandle * node_handle, unsigned long int measurements_publish_interval);
      ~RosserialPositionEncoder() = default;
      void setup();
      void enable();
      void disable();
      void spinOnce();
    private:
      void interrupt();
      void enableCallback(const std_msgs::Bool & message);
      void getAndPublishMeasurements();

  };


  template <int hall_a_pin, int hall_b_pin>
  RosserialPositionEncoder<hall_a_pin, hall_b_pin>::RosserialPositionEncoder(ros::NodeHandle * node_handle,
      unsigned long int measurements_publish_interval):
    node_handle_{node_handle},
    isr_wrapper_{},
    measurement_position_{0u},
    enable_subscriber_{"encoder/enable", &RosserialPositionEncoder::enableCallback, this},
    measurements_publisher_{"encoder/position", &measurements_message_},
    enable_{false},
    measurements_publish_interval_{measurements_publish_interval},
    measurements_last_published_{0UL}
  {
    ;
  }


  template <int hall_a_pin, int hall_b_pin>
  void RosserialPositionEncoder<hall_a_pin, hall_b_pin>::setup() {
    // Set up publishers and subscribers.
    node_handle_->advertise(measurements_publisher_);
    node_handle_->subscribe(enable_subscriber_);
    // Set up sensor.
    pinMode(hall_a_pin, INPUT);
    pinMode(hall_b_pin, INPUT);
    if(!isr_wrapper_.attachISR(std::bind(&RosserialPositionEncoder<hall_a_pin, hall_b_pin>::interrupt, this), RISING)) {
      pinMode(LED_BUILTIN, OUTPUT);
      while(true) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
      }
    }
  }


  template <int hall_a_pin, int hall_b_pin>
  void RosserialPositionEncoder<hall_a_pin, hall_b_pin>::enable() {
    if(enable_ == false) {
      measurements_last_published_ = millis() - measurements_publish_interval_;
    }
    enable_ = true;
  }


  template <int hall_a_pin, int hall_b_pin>
  void RosserialPositionEncoder<hall_a_pin, hall_b_pin>::disable() {
    enable_ = false;
  }


  template <int hall_a_pin, int hall_b_pin>
  void RosserialPositionEncoder<hall_a_pin, hall_b_pin>::spinOnce() {
    if(enable_) {
      auto current = millis();
      if(current >= measurements_last_published_ + measurements_publish_interval_
          || current < measurements_last_published_) {
        getAndPublishMeasurements();
        measurements_last_published_ += measurements_publish_interval_;    
      }
    }
  }


  template <int hall_a_pin, int hall_b_pin>
  void RosserialPositionEncoder<hall_a_pin, hall_b_pin>::interrupt() {
    if(digitalRead(hall_b_pin) == LOW) {
      -- measurement_position_;
    }
    else {
      ++ measurement_position_;
    }
  }


  template <int hall_a_pin, int hall_b_pin>
  void RosserialPositionEncoder<hall_a_pin, hall_b_pin>::enableCallback(const std_msgs::Bool & message) {
    if(message.data == true) {
      enable();
    }
    else {
      disable();
    }
  }


  template <int hall_a_pin, int hall_b_pin>
  void RosserialPositionEncoder<hall_a_pin, hall_b_pin>::getAndPublishMeasurements() {
    // Prepare message.
    measurements_message_.header.stamp = node_handle_->now();
    measurements_message_.pulse_count = measurement_position_;
    // TODO: add pulses per second (velocity) estimate.
    // Publish message.
    measurements_publisher_.publish(&measurements_message_);
  }

}


#endif