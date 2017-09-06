#!/usr/bin/env python
# Reading pressure data using LibMPSSE library
from mpsse import *
from decimal import Decimal

SIZE = 2		# 2 bytes MSB first
WCMD = "\x50"		# Write start address command
RCMD = "\x51"		# Read command

try:
	eeprom = MPSSE(I2C)

	print "%s initialized at %dHz (I2C)" % (eeprom.GetDescription(), eeprom.GetClock())

	eeprom.Start()
	eeprom.Write(WCMD)
	# Send Start condition to i2c-slave (Pressure Sensor)
	if eeprom.GetAck() == ACK:
		# ACK received,resend START condition and set R/W bit to 1 for read 
		eeprom.Start()
		eeprom.Write(RCMD)
	
		if eeprom.GetAck() == ACK:
			# ACK recieved, continue supply the clock to slave
			data = eeprom.Read(SIZE)
			eeprom.SendNacks()
			eeprom.Read(1)
		else:
			raise Exception("Received read command NACK2!")
	else:
		raise Exception("Received write command NACK1!")

	eeprom.Stop()
	# print(','.join(['{:d}'.format(x) for x in map(ord, data)]))
	hi = ord(data[0])
	lo = ord(data[1])
	response = (hi << 8) | lo

  	# Check if MSB is valid by reading MSB
  	status = response & 0xc000
 	status = status >> 14

 	if (status == 3 or status == 1):
   		 print("Diagnostic Fault")

	# Calibration for the pressure in psi. See https://sensing.honeywell.com/index.php?ci_id=45841
  	# Disregarding stale data (10)
  	if(status == 0):
  		output = Decimal((response - 1638) * 5) / Decimal((13107))
		print str(output) + " psi"

	# Closing connection with I2C-Slave
	eeprom.Close()



except Exception, e:
	print "MPSSE failure:", e
