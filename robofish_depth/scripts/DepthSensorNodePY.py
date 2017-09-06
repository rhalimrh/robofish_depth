#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import Adafruit_GPIO.FT232H as FT232H
from decimal import Decimal

def pressure_read():
	pub = rospy.Publisher('depth', Float64, queue_size = 10)
	rospy.init_node('depth_sensor', anonymous=True)

	# CHANGE THIS TO THE ONE IN CODE
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		response = i2c.readU16BE(0x28)
		status = response & 0xc000
  		status = status >> 14
		
		if (status == 3 or status == 1):
			print("Diagnostic Fault")

		if(status == 0):
  			output = Decimal((response - 1638) * 5) / Decimal((13107))

		if(status == 2):
			response = response & 0x3FFF
			output = Decimal((response - 1638) * 5) / Decimal((13107))

		rospy.loginfo(output)
		pub.publish(output)
		rate.sleep()
	

if __name__ == '__main__':
	try:
		# Temporarily disable FTDI serial drivers to use the FT232H device.
		FT232H.use_FT232H()
 
		# Create an FT232H device instance.
		ft232h = FT232H.FT232H()

		# Create an I2C device at address 0x28
		i2c = FT232H.I2CDevice(ft232h, 0x28)
		pressure_read()

	except rospy.ROSInterruptException:
		pass
