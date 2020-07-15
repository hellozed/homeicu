/*---------------------------------------------------------------------------------
 MAX30205 body temperature sensor 

 Modified by ZWang

 MAX30205, TDFN/8 package, $1.68 @1k, 0.1°C Accuracy (37°C to 39°C)
 16-Bit (0.00390625°C) Temperature Resolution
 2.7V to 3.3V Supply Voltage Range.

 MAX30208 reduces temperature measurement power by 50% (10-pin thin LGA package, 
 2mm x 2mm x 0.75mm, 1.7V to 3.6V), $1.31 (1000-up); The MAX30208EVSYS# evaluation 
 kit is available for $56. 
---------------------------------------------------------------------------------*/


//////////////////////////////////////////////////////////////////////////////////////////
//
//    Arduino library for the MAX30205 body temperature sensor breakout board
//
//    Author: Ashwin Whitchurch
//    Copyright (c) 2018 ProtoCentral
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   For information on how to use, visit https://github.com/protocentral/ProtoCentral_MAX30205
/////////////////////////////////////////////////////////////////////////////////////////
#include "firmware.h"
#include <Wire.h>

#if TEMP_SENSOR_MAX30325

#define MAX30205_ADDRESS        0x48  // 8bit address converted to 7bit

// Registers
#define MAX30205_TEMPERATURE    0x00  //  get temperature ,Read only
#define MAX30205_CONFIGURATION  0x01  //
#define MAX30205_THYST          0x02  //
#define MAX30205_TOS            0x03  //

#define MAX30205_READ_INTERVAL  500

typedef unsigned char uint8_t;

typedef enum{   	  // For configuration registers
  SHUTDOWN,    		  // shut down mode to reduce power consumption <3.5uA
  COMPARATOR,    	  // Bit 0 = operate OS in comparator mode, 1= INTERRUPT MODE
  OS_POLARITY,    	// Polarity bit ;Bit 0 = Active low output, Bit 1 = Active high
  FAULT_QUEUE_0,    // Fault indication bits
  FAULT_QUEUE_1,    // Fault indication bits
  DATA_FORMAT,      // Data Format
  TIME_OUT,         //  Time out
  ONE_SHOT          //  1= One shot, 0 = Continuos
}configuration;



class MAX30205
{
  public:
   float temperature = 0;      // Last temperature
   void  shutdown(void);   // Instructs device to power-save
   void  printRegisters(void); // Dumps contents of registers for debug
   bool  begin(void);
   float getTemperature(void);

  private:
    void    I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);
    void    I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);
    uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);   
};


float MAX30205::getTemperature(void)
{
  uint8_t readRaw[2] = {0};
  I2CreadBytes(MAX30205_ADDRESS,MAX30205_TEMPERATURE, &readRaw[0] ,2); // read two bytes
  int16_t raw = readRaw[0] << 8 | readRaw[1];  //combine two bytes
  temperature = raw  * 0.00390625;     // convert to temperature
  return  temperature;
}

void MAX30205::shutdown(void)
{
  uint8_t reg = I2CreadByte(MAX30205_ADDRESS, MAX30205_CONFIGURATION);  // Get the current register
  I2CwriteByte(MAX30205_ADDRESS, MAX30205_CONFIGURATION, reg | 0x80);
}

bool MAX30205::begin(void)
{
  I2CwriteByte(MAX30205_ADDRESS, MAX30205_CONFIGURATION, 0x00); //mode config
  I2CwriteByte(MAX30205_ADDRESS, MAX30205_THYST , 		 0x00); // set threshold
  I2CwriteByte(MAX30205_ADDRESS, MAX30205_TOS, 			 0x00); //
  return true;
}

void MAX30205::printRegisters(void)
{
  Serial.println(I2CreadByte(MAX30205_ADDRESS, MAX30205_TEMPERATURE),  BIN);
  Serial.println(I2CreadByte(MAX30205_ADDRESS, MAX30205_CONFIGURATION),  BIN);
  Serial.println(I2CreadByte(MAX30205_ADDRESS, MAX30205_THYST), BIN);
  Serial.println(I2CreadByte(MAX30205_ADDRESS, MAX30205_TOS), BIN);
}

// Wire.h read and write protocols
void MAX30205::I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t MAX30205::I2CreadByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t) 1);
  data = Wire.read();
  return data;
}

void MAX30205::I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	// Next send the register to be read. OR with 0x80 to indicate multi-read.
	Wire.write(subAddress);
	Wire.endTransmission(false);
	uint8_t i = 0;
	Wire.requestFrom(address, count);  // Read bytes from slave register address
	
	while (Wire.available())
	{
		dest[i++] = Wire.read();
	}
	
}


// -------------------------------------------

MAX30205  tempSensor;

float getTemperature()
{
  return tempSensor.getTemperature()*100;
}

boolean initTemperature()
{
  return tempSensor.begin();
}
#elif (TEMP_SENSOR_TMP117==false)

//if none of temperature sensor enabled
float getTemperature()    {return 0;}
boolean initTemperature() {return false;}

#endif //TEMP_SENSOR_MAX30325
