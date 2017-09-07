# robofish_depth
Reads pressure data from a Honeywell digital pressure sensor by a MT232H chip via I2C (written in various languages for testing purposes)

The previous pressure sensor used in the UW's NDCL Robofish project took up too much space in the RoboFish and is very outdated. The analog readings had to 
be converted to the digital readings via PIC A2D, which then are sent to the PC-104 on serial.
The new pressure sensor I'm using is the Honeywell Digital Output Pressure Sensor Model ABPDRRT005PG2A5, which is much smaller and cost-effective compared to 
the previous pressure sensor. They are calibrated and temperature compensated for sensor offset, sensitivity, temperature effects and accuracy errors using an ASIC. 
Furthermore, it avoids the hassle of converting analog signals to digital via A2D because it offers a digital output for reading pressure over the specified full scale pressure span. 
The digital output readings are read by an Adafruit FT232H Breakout via I2C. The MPSSE component of the FT232H can implement the I2C protocol which allows it to communicate with the Honeywell Pressure Sensor.

For testing purposes, I've written the code in Python (pressureRead folder) using a higher level GPIO library. To confirm the accuracy and
reliability of that code, I re-wrote the code in Python using a much lower-level LibMPSSE library. Both gave me the same exact results.

For calibration purposes, I've re-wrote the code in Arduino (Arduino folder) to output the pressure data onto an LCD screen while changing the
pressure using a water hose and known height.

The robofish_depth folder contains the pressure code integrated with ROS to allow communication between other nodes (Gyroscope, Temperature and etc.)
along with the simulations.
