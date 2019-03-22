#include <ros.h>
#include "rosserial_nmea_sentence_forwarder.hpp"


ros::NodeHandle node_handle;
earth_rover_firmware::RosserialNmeaSentenceForwarder<HardwareSerial>
    nmea_sentence_forwarder(&node_handle, &Serial1, 9600, 0, 1);


void setup() {
  node_handle.initNode();
  nmea_sentence_forwarder.setup();
}


void loop() {
  // nmea_sentence_forwarder.spinOnce();  // Handled in serialEvent1().
  node_handle.spinOnce();
}


void serialEvent1() {
  nmea_sentence_forwarder.spinOnce();
}