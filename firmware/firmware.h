#ifndef FIRMWARE_H__
#define FIRMWARE_H__

// This file is "forcedInclude" for every C/C++ code (NOT .ino code).
// The setting is in c_cpp_properties.json
// but this forceInclude does not work.

//#undef __APPLE__              // turn this on, if some error messages happen

#define TEST_PRINT      true    // show more print out
#define SIM_BATTERY     true    // use a potentiometer to simulate the bettery level circuit
 
/*
disable WEB_UPDATE to save: 20% of program space, 7% of dynamic memory
disable OTA_UPDATE to save:  2% of program space, 0% of dynamic memory
disable WiFi can significantly save battery power
*/
#define OTA_UPDATE      true   
#define WEB_UPDATE      false

#endif
