# Teensy 3.2 firmware for the Earth Rover

This package provides Teensy 3.2 firmware to control the Earth Rover via USB using the rosserial protocol. It currently provides a GPS and an IMU module.


## Modules

### GPS Module

### IMU Module

The IMU functionality is provided by an [Adafruit BNO055 Absolute Orientation Sensor](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview). The Teensy and the BNO055 breakout board communicate via I2C, so you'll have to connect the 3.3V, GND, SDA and SCL of the breakout board to your Teensy. By default, the first I2C interface's SDA and SCL lines are on pins 18 and 19. Configure the Wire (or i2c_t3) interface before initializing the sensor.

The rosserial node provides two publishers:

- `/bno055/imu` with the IMU's measurements (at 50 Hz) and
- `/bno055/calib_status` with the IMU's calibration status (at 1 Hz).

Both publishers can be enabled or disabled by sending a `true` or `false` bool message to `/bno055/enable`. They are disabled by default.

When the IMU is disabled while it is fully calibrated, the calibration offsets are stored in the Arduino's EEPROM memory. If stored offsets are available, they are restored after a reset.


## Dependencies

- [Arduino IDE](https://www.arduino.cc/en/Main/Software). I used version 1.8.8. Other versions might work too, but the one included in Ubuntu 18.04 LTS doesn't.
- [Teensyduino](https://www.pjrc.com/teensy/td_download.html). I used version 1.45.
- [Teensy Loader](https://www.pjrc.com/teensy/loader_cli.html).
- [ROS](http://www.ros.org/). I used Melodic Morenia on Ubuntu 18.04 LTS, but other versions might work too.
- [rosserial_arduino](http://wiki.ros.org/rosserial_arduino).
- [ros_teensy](https://github.com/mcgill-robotics/ros-teensy).


## Building

Include the package in a ROS workspace. Both building (messages, firmware...) and uploading the firmware is done using catkin_make.

Due to some internal details of rosserial_arduino's make_libraries.py script, building the package isn't as straightforward as I would like it to be. The problem is that to create our custom messages in the Arduino/Teensy ros_lib library, rosserial_arduino's make_libraries.py script needs to source the workspace's setup script, which isn't available until the build is finished. See [https://github.com/ros-drivers/rosserial/issues/239] for more details. 
The most elegant workaround I found is to exclude the firmware from the default catkin_make (or CMake) target and build it manually afterwards.

So, to build the package including the firmware for the Arduino Micro, run:

- `export arduino_location="/opt/arduino-1.8.8"`
- `. /opt/ros/melodic/setup.bash` (or the version for your favourite shell)
- `rm -rf build devel`
- `catkin_make` (to build everything except the firmware)
- `. ./devel/setup.bash` (or the version for your favourite shell)
- `catkin_make earth_rover_firmware_firmware_teensy32_earth_rover_firmware_teensy32_Firmware` (to build the firmware)
- `teensy_loader_cli -mmcu=mk20dx256 -v -w ./build/earth_rover_firmware/firmware_teensy32/bin/earth_rover_firmware_teensy32.elf.hex` (to upload the firmware to the Teensy 3.2)


## Running


## License

MIT license, see LICENSE.md for details.
