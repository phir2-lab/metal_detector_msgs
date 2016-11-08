# About the Metal Detector ROS node

The metal detector node has been designed to work directly with the rosserial package. Thus, the firmware posted in the 'electronics/arduino_firmware' folder of this branch implements the node, while this launch only invokes the rosserial interface.

## Dependencies
* [Rosserial Package](https://github.com/ros-drivers/rosserial.git)

## Troubleshooting
* Unable to sync with device error
This is probably due to the Arduino being attributed to a different serial port by the OS. Simply change the '_port' parameter in the node's arguments to the correct port.