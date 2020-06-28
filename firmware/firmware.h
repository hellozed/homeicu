#ifndef FIRMWARE_H1__
#define FIRMWARE_H1__

// This file is "forcedInclude" for every C/C++ code (NOT .ino code).
// The setting is in c_cpp_properties.json
// but this forceInclude does not work.
// "${workspaceFolder}/firmware/firmware.h"
   
//#undef __APPLE__                // turn this on, if some error messages happen

/*---------------------------------------------------------------------------------
  Test Parameters
---------------------------------------------------------------------------------*/
#define TEST_PRINT      true    // show  more print out
#define SIM_BATTERY     true    // potentiometer A simulates the bettery voltage detction
#define SIM_TEMPERATURE true    // potentiometer B to simulate the temperature sensor
#define JOYTICK_TEST    true    // use joy stick y to simulate 
#define ECG_BLE_TEST    true    // send fake ecg data to BLE
#define SIM_PPG         true    // use joy stick x to simulate PPG
/*---------------------------------------------------------------------------------
  PIN number defined by ESP-WROOM-32 IO port number
---------------------------------------------------------------------------------*/
const int ADS1292R_CS_PIN    = 13;
const int ADS1292R_START_PIN = 14;
const int ADS1292R_DRDY_PIN  = 26;
const int ADS1292R_PWDN_PIN  = 27;

const int AFE4490_CS_PIN    = 21; 
const int AFE4490_DRDY_PIN  = 39; 
const int AFE4490_PWDN_PIN  = 4;
const int PUSH_BUTTON_PIN   = 0;
const int LED_PIN           = 2;
const int SENSOR_VP_PIN     = 36; 

#if JOYTICK_TEST
const int JOYY_PIN          = 34;   //GPIO34 ADC
#endif

#if SIM_PPG
const int JOYX_PIN          = 32;   //GPIO32 ADC
#endif 

#if SIM_TEMPERATURE
const int SENSOR_TEMP       = 35;   //GPIO35 ADC
#endif
/*
default SPI VSPI: 
                      MOSI  = 23
                      MISO  = 19
                      SCK   = 18
                      SS    =  5 (not connected)
 
I2C 
                      CLK   = 22
                      SDA   = 25
*/

#define ECG_BUFFER_SIZE         5  
#define ECG_SAMPLE_RATE         125
  
#define PPG_BUFFER_SIZE         5
#define HVR_ARRAY_SIZE          13

#define HISTGRM_DATA_SIZE       12*4
#define HISTGRM_PERCENT_SIZE    HISTGRM_DATA_SIZE/4
 
union FloatByte {               // 1 float type = 4 byte 
  float f;
  uint8_t b[4];
};
/* 
Blank project with OTA take 39% program space, 13% of dynamic memory.
+ WEB_UPDATE take:          20% program space,  7% of dynamic memory
disable OTA update to save:  2% program space,  0% of dynamic memory
disable WiFi can significantly save battery power
*/
#define WEB_UPDATE      false   // *false

#endif //FIRMWARE_H__