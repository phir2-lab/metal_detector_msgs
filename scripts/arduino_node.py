#!/usr/bin/env python

import time
import rospy
import serial
import argparse
import serial.tools.list_ports
from serial import SerialException
from metal_detector_msgs.msg import Coil

# ROS Parameters
pub_freq = 10
pub_topic = 'detector'
ser = serial.Serial()

# Coil message
coil_msg = Coil()
coil_msg.header.frame_id = 'Coil'
coil_msg.header.seq = 0

# Argument parser
parser = argparse.ArgumentParser()

# ROSLAUNCH arguments
parser.add_argument("__name",
                    help='ROSLAUNCH name for the node')
parser.add_argument("__log",
                    help='ROSLAUNCH log for the node')

# Regular arguments
parser.add_argument("-b", "--baud",
                    default=9600,
                    help='defines baud rate of serial port')
parser.add_argument("--vid",
                    default='2341',
                    help='vendor ID of Arduino device (default is 2341 for Arduino UNO)')

args = parser.parse_args()

def find_arduino(vid='2341'):
	'''
	Sweeps all available serial ports and finds the arduino port via its 
	Vendor ID.

	Arguments:
		vid - Vendor ID for the Arduino (2341 is the default for UNO)

	Returns:
		port - String with the arduino port
	'''

	# Make a list of all available ports on the system
	available_ports = list(serial.tools.list_ports.comports())
	
	# Sweep all ports
	for port in available_ports:
		for string in port:			# Sweep all strings in a given port list
			if vid in string:		# If any of these strings contain the VID
				return port[0]		# Return the first string, which is the port name

	# If no arduino has been found, raise a serialexception
	raise SerialException('[ERROR] Could not find the Arduino - is it plugged in?')


def detector():
	'''
	Gets values from serial and publishes them as Coil.msg in the detector topic
	'''
	pub = rospy.Publisher('metal_detector', Coil, queue_size=10)
	rospy.init_node(pub_topic, anonymous=True)
	rate = rospy.Rate(pub_freq) # 10hz
    
	# Catch serial errors
	try:
	    ser.port = find_arduino(args.vid)
	    ser.baudrate = args.baud
	    ser.open()
	    # If successfully opened serial port
	    if ser.isOpen():
			rospy.loginfo("Connected to device at %s", ser.port)
			rospy.loginfo("Publishing at topic \'%s\' at a rate of %d hz", pub_topic, pub_freq)
            buf_str = ''
            last_received = ''
            # Do this until node is shutdown
            while not rospy.is_shutdown():
            	# Get current time for coil message
                coil_msg.header.stamp = rospy.Time.from_sec(time.time())
                
                # Parse the buffer string and get the last available data (in pairs)
                buf_str = buf_str + ser.read(ser.inWaiting())
                if '\n' in buf_str:
                	lines = buf_str.split('\n')
                	last_received = lines[-2]
                	buf_str = lines[-1]

                data = last_received.split(',')

                # Catch some silly errors (when serial sends two ","s for example)
                try:
                    coil_msg.channel = ( int(data[0]), int(data[1]) )
                except (ValueError, IndexError):
                    pass
                pub.publish(coil_msg)
                rate.sleep()

            ser.close()

	except SerialException as exp:
		print exp
		pass
    
if __name__ == '__main__':    
    try:
        detector()
    except rospy.ROSInterruptException:
        pass