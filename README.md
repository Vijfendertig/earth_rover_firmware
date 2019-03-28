# Teensy 3.2 firmware for the Earth Rover

This package provides Teensy 3.2 firmware to control the Earth Rover via USB using the rosserial protocol. It currently provides a GPS module based on a MTK3339 sensor, an IMU module based on a BNO055 sensor and a servo module.


## Modules

### GPS Module

The GPS functionality is provided by an [Adafruit Ultimate GPS Breakout](https://www.adafruit.com/product/746) based on a MTK3339 GPS sensor.

The rosserial GPS module provides one publisher:

- `/mtk3339/nmea_sentence` with the NMEA sentences published by the GPS sensor.

This publisher can be enabled or disabled by sending a `true` or `false` bool message to `/mtk3339/enable`. It is disabled by default.

The standard `nmea_navsat_driver` package provides a `nmea_topic_driver` node which translates the NMEA sentences to standard `sensor_msgs/NavSatFix`, `geometry_msgs/TwistStamped` and `sensor_msgs/TimeReference` ROS messages.


### IMU Module

The IMU functionality is provided by an [Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout ](https://www.adafruit.com/product/2472) based on a Bosch BNO055 IMU sensor. The Teensy and the BNO055 breakout board communicate via I2C, so you'll have to connect the 3.3V, GND, SDA and SCL of the breakout board to your Teensy. Configure the Wire (or i2c_t3) interface before initializing the sensor. The current firmware uses the first I2C interface's SDA and SCL lines on pins 18 and 19. 

The rosserial IMU module provides two publishers:

- `/bno055/imu` with the IMU's measurements (at 50 Hz) and
- `/bno055/calib_status` with the IMU's calibration status (at 1 Hz).

Both publishers can be enabled or disabled by sending a `true` or `false` bool message to `/bno055/enable`. They are disabled by default.

When the IMU is disabled while it is fully calibrated, the calibration offsets are stored in the Arduino's EEPROM memory. If stored offsets are available, they are restored after a reset.

This package also provides a `imu_publisher_node` node which translates the compact `Bno055Measurements` and `Bno055CalibrationStatus` messages to standard `sensor_msgs/Imu` ROS messages.


### Servo module

The rosserial servo module provides three subscribers:

- `/servo/control` to set the servo's position relative (-1000 = minimum, 0 = center and +1000 = maximum) to the configured pulse width limits.
- `/servo/control_raw` to send absolute pulse widths (in µs) to the servo (for most servos, 1000µs = minimum and 2000µs = maximum).
- `/servo/configure` to configure the servo's initial, minimum, center and maximum pulse width.

The pin numbers are given as template parameters. The current firmware uses pins 20, 21, 22 and 23.


## Dependencies

- [Arduino IDE](https://www.arduino.cc/en/Main/Software). I use version 1.8.8. Other versions might work too, but the one included in Ubuntu 18.04 LTS doesn't.
- [Teensyduino](https://www.pjrc.com/teensy/td_download.html). I use version 1.45.
- [Teensy Loader](https://www.pjrc.com/teensy/loader_cli.html).
- [ROS](http://www.ros.org/). I use Melodic Morenia on Ubuntu 18.04 LTS, but other versions might work too.
- [rosserial_arduino](http://wiki.ros.org/rosserial_arduino).
- [ros_teensy](https://github.com/mcgill-robotics/ros-teensy).


## Building

Include the packages in a ROS workspace. Both building (messages, firmware...) and uploading the firmware is done using `catkin build` (or `catkin_make`).

To build the package using `catkin build`, run:

- `export arduino_location="/opt/arduino-1.8.8"`
- `. /opt/ros/melodic/setup.bash` (or the version for your favourite shell)
- `rm -rf build devel logs`
- `catkin build`

Due to some internal details of rosserial_arduino's make_libraries.py script, building the package with `catkin_make` isn't as straightforward as I would like it to be. The problem is that to create our custom messages in the Arduino/Teensy ros_lib library, rosserial_arduino's make_libraries.py script needs to source the workspace's setup script, which isn't available until the build is finished. See [https://github.com/ros-drivers/rosserial/issues/239] for more details. 
The most elegant workaround I found is to exclude the firmware from the default `catkin_make` target (remove the `ALL` from the `rosserial_add_client_target` command) and build it manually afterwards.

So, to build the package including the firmware for the Teensy 3.2 using `catkin_make`, run:

- `export arduino_location="/opt/arduino-1.8.8"`
- `. /opt/ros/melodic/setup.bash` (or the version for your favourite shell)
- `rm -rf build devel`
- `catkin_make` (to build everything except the firmware)
- `. ./devel/setup.bash` (or the version for your favourite shell)
- `catkin_make earth_rover_firmware_firmware_teensy32_earth_rover_firmware_teensy32_Firmware` (to build the firmware)
- `teensy_loader_cli -mmcu=mk20dx256 -v -w ./build/earth_rover_firmware/firmware_teensy32/bin/earth_rover_firmware_teensy32.elf.hex` (to upload the firmware to the Teensy 3.2)


## Running

This package provides a `earth_rover_firmware.launch` launch file to start the rosserial interface node, enables all modules and starts the standard message (re)publishers.


## License

MIT license, see LICENSE.md for details.
