#ifndef FIRMWARE_H1__
#define FIRMWARE_H1__

// This file is "forcedInclude" for every C/C++ code (NOT .ino code).
// The setting is in c_cpp_properties.json
// but this forceInclude does not work.
// "${workspaceFolder}/firmware/firmware.h"
   
//#undef __APPLE__                // turn this on, if some error messages happen

#define ECG_BUFFER_SIZE         25  
#define ECG_SAMPLE_RATE         125
  
#define PPG_BUFFER_SIZE         25
#define HVR_ARRAY_SIZE          13

#define HISTGRM_DATA_SIZE       12*4
#define HISTGRM_PERCENT_SIZE    HISTGRM_DATA_SIZE/4


#define TEST_PRINT      true    // show  more print out
#define SIM_BATTERY     true    // potentiometer A simulates the bettery voltage detction
#define SIM_TEMPERATURE true    // potentiometer B to simulate the temperature sensor
#define JOY_TEST        true    // *false, use Joy stick to simulate heart ecg
#define ECG_BLE_TEST    true    // send fake ecg data to BLE
 
/* 
Blank project with OTA take 39% program space, 13% of dynamic memory.
+ WEB_UPDATE take:          20% program space,  7% of dynamic memory
disable OTA update to save:  2% program space,  0% of dynamic memory
disable WiFi can significantly save battery power
*/
#define WEB_UPDATE      false   // *false

#endif //FIRMWARE_H__