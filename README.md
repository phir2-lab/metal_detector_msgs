# Open Hardware, ROS-enable metal detector

This is a ROS package for the metal detector used in the [HRATC2017](http://inf.ufrgs.br/hratc2017/HRATC2017/Welcome.html) robot programming competition. The detector is based on open-hardware and software design.

## Usage

Program your Arduino with the firmware located in the "electronics/arduino_firmware" folder, connect it to your computer and run the launch file "detector.launch" with:

```
roslaunch metal_detector_msgs detector.launch
```

Or run the "arduino_node.py" node with:

```
rosrun metal_detector_msgs arduino_node.py [args]
```

The arguments (optional) to be passed to the node are:
* -b, --baud : Serial BAUD rate of the Arduino (default is 9600)
* --vid 	 : Vendor ID of the Arduino (default is 2341 for Arduino UNO)

This can also be seen when passing "-h" as argument for the node.

## Dependencies
* [ROS Indigo](http://wiki.ros.org/indigo)
* [pySerial](http://pyserial.readthedocs.io/en/latest/pyserial.html)

## Installation

Clone this package to your [Catkin Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) "src" folder and [build your catkin packages](http://wiki.ros.org/catkin/Tutorials/using_a_workspace).

## Topic

The node receives frequency measurements from an Arduino via Serial Communication and publishes a topic of the Coil message type, of which the definition can be seen in the "msg" folder. Please see the readme of the "msg" folder for more information regarding interface with the coils.

## Parameters

The package uses ROS' parameter server to get and set its calibration parameters. They are defined as:

* calibration/sat : Saturation value for the coils in Hz
* calibration/ref : The desired reference value (when coils saturate)

By default, these parameters are set to "1.0".
The node also caps the readings to the reference value. In other words, it returns readings in the range of [0 - ref].

The parameters can be set statically via the "detector.launch" file. Just change the <param> tags accordingly, as seen in the example below:

```
<param name="calibration/sat" type="double" value="1853" /> 
```

Alternatively, to set the parameters dynamically, use the command below after launching the detector node:

```
rosparam set calibration/sat 1853
```

To check the value of a parameter, use the "rosparam get" command:
```
rosparam get calibration/sat
```

## Package organization

- model: has the Solidworks and Gazebo models for the metal detector;
- eletronics: it has the hardware design, eletronics, and Arduino programming;
- other dirs: ROS-related files such as launch, node, msg, etc.

## Troubleshooting

* Can't connect to my Arduino

The "arduino_node.py" sweeps your /dev/ folder for all available serial connections and finds the Arduino automatically via its Vendor ID (vid). The Arduino's vid can be set through the "--vid" argument when running the node, or by changing the "args" parameter in the launch file. To find out what your Arduino vid is, connect it to a computer and run the "lsusb" command in a terminal. The vid will be there, like this:

<pre>
Bus 002 Device 006: ID 0cf3:e005 Atheros Communications, Inc. 
Bus 002 Device 003: ID 0c45:6712 Microdia 
Bus 002 Device 009: ID <b>2341</b>:0043 Arduino SA Uno R3 (CDC ACM)
                          ^
                       [This is your VID]   
</pre>

## Credits

- The original eletronics board was designed by [LucasLab](https://translate.google.com/translate?sl=auto&tl=en&js=y&prev=_t&hl=pt-BR&ie=UTF-8&u=http%3A%2F%2Fwww.lucaslab.grandhost.pl%2Flucaslab%2Fwykrywacz_PI.htm&edit-text=)
- The integration of the eletronics board with Arduino is by Augusto Bergamin;
- The Arduino firmware is by Augusto Bergamin and Renan Maidana;
- The ROS node design and ROS launch is by Renan Maidana;
- Conversion from SolidWorks to Gazebo by Renan Maffei and Mathias Mantelli.
- Readmes edits by Alexandre Amory.
