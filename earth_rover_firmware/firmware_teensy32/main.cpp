#include <i2c_t3.h>
#include <ros.h>
#include "rosserial_nmea_sentence_forwarder.hpp"
#include "rosserial_adafruit_bno055.hpp"
#include "rosserial_servo_controller.hpp"


ros::NodeHandle node_handle;
earth_rover_firmware::RosserialAdafruitBNO055<i2c_t3>
    adafruit_bno055(&node_handle,
                    &Wire, earth_rover_firmware::AdafruitBNO055<i2c_t3>::BNO055_ADDRESS_A,
                    20ul, 1000ul,
                    0u, 8u);
earth_rover_firmware::RosserialNmeaSentenceForwarder<HardwareSerial>
    nmea_sentence_forwarder(&node_handle,
                            &Serial1, 9600, 0, 1);
earth_rover_firmware::RosserialServoController<20, 21, 22, 23>
    servo_controller(&node_handle,
                     512u);


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // Initialise I2C bus.
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000, I2C_OP_MODE_ISR);
  // Initialise ROS node.
  node_handle.initNode();
  adafruit_bno055.setup();
  nmea_sentence_forwarder.setup();
  servo_controller.setup();
  digitalWrite(LED_BUILTIN, LOW);
}


void loop() {
  // nmea_sentence_forwarder.spinOnce();  // Handled in serialEvent1().
  adafruit_bno055.spinOnce();
  node_handle.spinOnce();
}


void serialEvent1() {
  nmea_sentence_forwarder.spinOnce();
}