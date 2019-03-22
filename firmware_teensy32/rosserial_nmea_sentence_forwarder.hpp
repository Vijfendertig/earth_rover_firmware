#ifndef __ROSSERAIL_NMEA_SENTENCE_FORWARDER
#define __ROSSERAIL_NMEA_SENTENCE_FORWARDER


#include <ros.h>
#include <nmea_msgs/Sentence.h>


namespace earth_rover_firmware {


  template <typename SerialDevice>
  class RosserialNmeaSentenceForwarder {
    private:
      SerialDevice * gps_device_;
      unsigned gps_baudrate_;
      int gps_rx_pin_;
      int gps_tx_pin_;
      char buffer_[128];
      unsigned buffer_next_index_;
      ros::NodeHandle * node_handle_;
      ros::Publisher nmea_sentence_publisher_;
      nmea_msgs::Sentence nmea_sentence_message_;
      bool enable_forwarding_;
    public:
      RosserialNmeaSentenceForwarder(ros::NodeHandle * node_handle, SerialDevice * gps_device,
          unsigned gps_baudrate, int gps_rx_pin = -1, int gps_tx_pin = -1);
      ~RosserialNmeaSentenceForwarder() = default;
      void setup();
      void enable() {enable_forwarding_ = true; };
      void disable() {enable_forwarding_ = false; };
      void spinOnce();
  };


  template<typename SerialDevice>
  RosserialNmeaSentenceForwarder<SerialDevice>::RosserialNmeaSentenceForwarder(ros::NodeHandle * node_handle,
      SerialDevice * gps_device, unsigned gps_baudrate, int gps_rx_pin, int gps_tx_pin):
    gps_device_{gps_device},
    gps_baudrate_{gps_baudrate},
    gps_rx_pin_{gps_rx_pin},
    gps_tx_pin_{gps_tx_pin},
    buffer_next_index_{0},
    node_handle_{node_handle},
    nmea_sentence_publisher_{"nmea_sentence", &nmea_sentence_message_},
    enable_forwarding_{true}
  {
    ;
  }


  template<typename SerialDevice>
  void RosserialNmeaSentenceForwarder<SerialDevice>::setup() {
    // Initialise UART.
    if(gps_rx_pin_ >= 0) {
      gps_device_->setRX(gps_rx_pin_);
    }
    if(gps_tx_pin_ >= 0) {
      gps_device_->setTX(gps_tx_pin_);
    }
    gps_device_->begin(gps_baudrate_, SERIAL_8N1);
    // Initialise ROS publisher.
    node_handle_->advertise(nmea_sentence_publisher_);
    nmea_sentence_message_.header.frame_id = "earth_rover_gps";
  }


  template<typename SerialDevice>
  void RosserialNmeaSentenceForwarder<SerialDevice>::spinOnce() {
    while(gps_device_->available()) {
      // If starting a new sentence, store the current timestamp in the sentence message.
      if(buffer_next_index_ == 0) {
        nmea_sentence_message_.header.stamp = node_handle_->now();
      }
      // Store received character.
      buffer_[buffer_next_index_ ++] = char(gps_device_->read() & 0x00ff);
      // If we received a \r\n line terminator, send the message.
      if(buffer_[buffer_next_index_ - 2] == '\r' && buffer_[buffer_next_index_ - 1] == '\n') {
        buffer_[buffer_next_index_ - 2] = '\0';
        nmea_sentence_message_.sentence = buffer_;
        nmea_sentence_publisher_.publish(&nmea_sentence_message_);
        buffer_next_index_ = 0;
      }
      // If the buffer overflows, send the message to enable debugging.
      else if(buffer_next_index_ >= sizeof(buffer_) - 1) {
        buffer_[buffer_next_index_] = '\0';
        nmea_sentence_message_.sentence = buffer_;
        nmea_sentence_publisher_.publish(&nmea_sentence_message_);
        buffer_next_index_ = 0;
      }
    }
  }

}


#endif