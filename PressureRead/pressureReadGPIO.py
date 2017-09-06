# Reads 100 values of pressure in psi
# Reading pressure data using GPIO Library
import Adafruit_GPIO.FT232H as FT232H
from decimal import Decimal
 
# Temporarily disable FTDI serial drivers to use the FT232H device.
FT232H.use_FT232H()
 
# Create an FT232H device instance.
ft232h = FT232H.FT232H()

# Create an I2C device at address 0x28
i2c = FT232H.I2CDevice(ft232h, 0x28)

# Reading the first two bytes for pressure output
# Last two bytes measures temperature, which is not needed

count = 0

while(count < 100):
  # Read a 16 byte unsigned big endian value from register 0x028
  response = i2c.readU16BE(0x28)

  # 0x8656 = 0x1000 0110 0101 0110
  
  # Check if MSB is valid
  # 00 and 10 means valid stable data
  # 01 means command mode and 11 means diagnostic fault
  # 1000011001010110 & 1100000000000000 >> 14 to read 2 MSB
  # = 0000 0000 0000 0010 (MSB)
  status = response & 0xc000
  status = status >> 14

  if (status == 3 or status == 1):
    print("Diagnostic Fault")

  # Calibration for the pressure in psi. See https://sensing.honeywell.com/index.php?ci_id=45841
  # Disregarding stale data (10)
  if(status == 0):
  	output = Decimal((response - 1638) * 5) / Decimal((13107))
	print str(output) + " psi"
  	count = count + 1
print "Done!" 
