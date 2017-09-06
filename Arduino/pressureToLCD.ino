/*
Code By: Ryan Halim
----------------------------
The pressure sensor being used is the ABPDRRT005PG2A5. It is of a
Honeywell design and is a fine sensor. Documentation of the
device can be found at the sight below.

http://sensing.honeywell.com/index.php%3Fci_id%3D45841

Since I2C is being used to connect the sensor and the arduino, the
setup is fairly simple. If the ports are faced away, the input
pins look like the following

           ^_^_
        3_|    |_4
        2_|    |_5
        1_|____|_6

Pin 1 is GND, Pin 2 Input (Power), ignore Pin 3 and 4, Pin 5 is 
SDA and Pin 6 is SCL. To connect to the arduino the GND obviously
goes to ground and 5V power goes to power. The SDA of the sensor
connects with A4 of the arduino and SCL connects with A5 of the
arduino. For the I2C connection to work there needs to be a pull
up resistor (1kOhm) connection between Pin 5 (SDA) to power and
another resistor (1kOhm) between Pin 6 (SCL) and power.


To use this code, after plugging into Arduino it requires serial input from user. An input
of 1 prints the average output value of the sensor for 1000 samples. An input of 0 prints
the output value of the sensor, the number of bits being returned by the sensor, and the 8-bit 
number.

The pressure data will be outputted on an LCD screen
*/

// Include the Wire Library
#include <Wire.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Set the address of the temp
#define press_address 0x48

unsigned long overallP;
int count;
boolean start;
boolean printData;

void setup() {
  lcd.begin(16, 2);
  
  Serial.begin(9600);
  start = false;
  Wire.begin(); // Wakes up I2C bus
}

void loop() {  
  
  byte aa,bb;
  getdata(&aa,&bb);    // Gets the first two bytes from sensor
  unsigned int bigData = aa;  // First byte is bulk of height
  unsigned int smallData = bb;  // Second byte is 

  unsigned long pressure1 = (bigData * 256) + smallData;
  Serial.println(pressure1);
  double pressure = ((pressure1 - 1638.0) * 5.0) / 13107.0;
  Serial.println(pressure, 5);
  lcd.println(pressure, 5);
  delay(3000); 
  lcd.clear();
  
}


void getdata(byte *a, byte *b)
{
  Wire.beginTransmission(press_address);
  Wire.requestFrom(press_address,2);//Sends content of first two registers
  *a = Wire.read(); // Measures number of bytes of pressure sensor
  *b = Wire.read(); // 8 bit count of pressure (counts to 256 then adds to total number of bytes).
}


