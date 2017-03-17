#!/usr/bin/env python

import os
import sys
import time
import rospy
import signal
import serial
import argparse
import serial.tools.list_ports
from serial import SerialException
from metal_detector_msgs.msg import Coil

# ROS Parameters
pub_freq = 10
pub_topic = 'metal_detector'
ser = serial.Serial()

# Coil message
coil_msg = Coil()
coil_msg.header.frame_id = 'Coils'
coil_msg.header.seq = 0

# Argument parser
parser = argparse.ArgumentParser()

# Regular arguments
parser.add_argument("-b", "--baud",
                    default=9600,
                    help='defines baud rate of serial port')
parser.add_argument("--vid",
                    default='2341',
                    help='vendor ID of Arduino device (default is 2341 for Arduino UNO)')

# Parse known arguments because ROSLAUNCH passes unknown arguments to node
args, unknown = parser.parse_known_args()

def handle_ctrl_c(signal, frame):
	ser.close()
	sys.exit(130)

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

	return None

def connect(timeout=10):
	'''
	Attempts to connect to a serial port specified by "ser.port".

	Arguments:
		timeout - Time to display message "Could not find Arduino"
	'''
	t1 = time.time()
	while ser.port == None:
		t2 = time.time()
		ser.port = find_arduino(args.vid)
		ser.baudrate = args.baud
		if (t2 - t1) > timeout:
			rospy.loginfo("Could not find Arduino. Retrying...")
			t1 = t2
	
	try:	
		ser.open()
		rospy.loginfo("Connected to device at %s", ser.port)
	except:
		pass

def detector():
	'''
	Gets values from serial and publishes them as Coil.msg in the detector topic
	'''
	pub = rospy.Publisher('metal_detector', Coil, queue_size=10)
	rospy.init_node(pub_topic, anonymous=True)
	rate = rospy.Rate(pub_freq) # 10hz

	rospy.loginfo("Connecting...")
	while not ser.isOpen():
		connect(10)

	rospy.loginfo("Publishing at topic \'%s\' at a rate of %d hz", pub_topic, pub_freq)
	buf_str = ''
	last_received = ''
	# Do this until node is shutdown
	while not rospy.is_shutdown():
		# Get current time for coil message
		coil_msg.header.stamp = rospy.Time.from_sec(time.time())

		# Get calibration parameters from parameter server
		try:
			sat = float(rospy.get_param('/metal_detector/calibration/sat'))
		except KeyError:	# rospy will raise this error if the parameter is not found
			sat = 1.0		# default to 1.0
		try:
			ref = float(rospy.get_param('/metal_detector/calibration/ref'))
		except KeyError: 	# rospy will raise this error if the parameter is not found
			ref = 1.0		# default to 1.0

		# Parse the buffer string and get the last available data (in pairs)
		try:
			buf_str += ser.read(ser.inWaiting())
		except IOError:
			rospy.loginfo("Lost connection to Arduino! Trying to reestablish connection...")
			ser.close()
			ser.port = None
			while not ser.isOpen():
				connect(5)
			rospy.loginfo("Publishing at topic \'%s\' at a rate of %d hz", pub_topic, pub_freq)
				
		if '\n' in buf_str:
			lines = buf_str.split('\n')
			last_received = lines[-2]
			buf_str = lines[-1]

		data = last_received.split(',')

		# Catch some silly errors (when serial sends two ","s for example)
		try:
			# Normalize the data comming from the coils, and caps them to the interval [0 - ref]
			coil_msg.left_coil = max(0, min(ref*float(data[0])/sat, ref))
			coil_msg.right_coil = max(0, min(ref*float(data[1])/sat, ref))
		except (ValueError, IndexError):
			pass
		pub.publish(coil_msg)
		rate.sleep()
	ser.close()

if __name__ == '__main__':
	signal.signal(signal.SIGINT, handle_ctrl_c)
	try:
		detector()
	except rospy.ROSInterruptException:
		pass
