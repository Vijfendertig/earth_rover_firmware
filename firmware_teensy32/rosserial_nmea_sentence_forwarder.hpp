//! NMEA sentence (re)publisher for serial GPS device.
/*!
 *  This class republishes NMEA sentences received from a serial GPS device to the ROS message bus via
 *  rosserial.
 * 
 *  \file
 * 
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __ROSSERAIL_NMEA_SENTENCE_FORWARDER
#define __ROSSERAIL_NMEA_SENTENCE_FORWARDER


#include <ros.h>
#include <nmea_msgs/Sentence.h>
#include <std_msgs/Bool.h>


namespace earth_rover_firmware {

  template <typename SerialDevice>
  class RosserialNmeaSentenceForwarder {
    private:
      SerialDevice * serial_device_;
      unsigned serial_baudrate_;
      int serial_rx_pin_;
      int serial_tx_pin_;
      char buffer_[128];
      unsigned buffer_next_index_;
      ros::NodeHandle * node_handle_;
      ros::Subscriber<std_msgs::Bool, RosserialNmeaSentenceForwarder> enable_subscriber_;
      ros::Publisher nmea_sentence_publisher_;
      nmea_msgs::Sentence nmea_sentence_message_;
      bool enable_forwarding_;
    public:
      RosserialNmeaSentenceForwarder(ros::NodeHandle * node_handle, SerialDevice * serial_device,
          unsigned serial_baudrate, int serial_rx_pin = -1, int serial_tx_pin = -1);
      ~RosserialNmeaSentenceForwarder() = default;
      void setup();
      void enable() { enable_forwarding_ = true; };
      void disable() { enable_forwarding_ = false; };
      void spinOnce();
    private:
      void enableCallback(const std_msgs::Bool & message);
  };


  template<typename SerialDevice>
  RosserialNmeaSentenceForwarder<SerialDevice>::RosserialNmeaSentenceForwarder(ros::NodeHandle * node_handle,
      SerialDevice * serial_device, unsigned serial_baudrate, int serial_rx_pin, int serial_tx_pin):
    serial_device_{serial_device},
    serial_baudrate_{serial_baudrate},
    serial_rx_pin_{serial_rx_pin},
    serial_tx_pin_{serial_tx_pin},
    buffer_next_index_{0},
    node_handle_{node_handle},
    enable_subscriber_{"mtk3339/enable", &RosserialNmeaSentenceForwarder::enableCallback, this},
    nmea_sentence_publisher_{"mtk3339/nmea_sentence", &nmea_sentence_message_},
    enable_forwarding_{false}
  {
    ;
  }


  template<typename SerialDevice>
  void RosserialNmeaSentenceForwarder<SerialDevice>::setup() {
    // Initialise serial device.
    if(serial_rx_pin_ >= 0) {
      serial_device_->setRX(serial_rx_pin_);
    }
    if(serial_tx_pin_ >= 0) {
      serial_device_->setTX(serial_tx_pin_);
    }
    serial_device_->begin(serial_baudrate_, SERIAL_8N1);
    // Initialise ROS publisher.
    node_handle_->advertise(nmea_sentence_publisher_);
    nmea_sentence_message_.header.frame_id = "mtk3339_link";
    node_handle_->subscribe(enable_subscriber_);
  }


  template<typename SerialDevice>
  void RosserialNmeaSentenceForwarder<SerialDevice>::spinOnce() {
    while(serial_device_->available()) {
      // If starting a new sentence, store the current timestamp in the sentence message.
      if(buffer_next_index_ == 0) {
        nmea_sentence_message_.header.stamp = node_handle_->now();
      }
      // Store received character.
      buffer_[buffer_next_index_ ++] = char(serial_device_->read() & 0x00ff);
      // If we received a \r\n line terminator, send the message.
      if(buffer_[buffer_next_index_ - 2] == '\r' && buffer_[buffer_next_index_ - 1] == '\n') {
        buffer_[buffer_next_index_ - 2] = '\0';
        if(enable_forwarding_) {
          nmea_sentence_message_.sentence = buffer_;
          nmea_sentence_publisher_.publish(&nmea_sentence_message_);
        }
        buffer_next_index_ = 0;
      }
      // If the buffer overflows, send the message to enable debugging.
      else if(buffer_next_index_ >= sizeof(buffer_) - 1) {
        buffer_[buffer_next_index_] = '\0';
        if(enable_forwarding_) {
          nmea_sentence_message_.sentence = buffer_;
          nmea_sentence_publisher_.publish(&nmea_sentence_message_);
        }
        buffer_next_index_ = 0;
      }
    }
  }


  template<typename SerialDevice>
  void RosserialNmeaSentenceForwarder<SerialDevice>::enableCallback (const std_msgs::Bool & message) {
    if(message.data == true) {
      enable();
    }
    else {
      disable();
    }
  }

}


#endif