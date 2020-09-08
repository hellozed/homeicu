#ifndef __PUBLIC_H__
#define __PUBLIC_H__

#include <Arduino.h>

/*---------------------------------------------------------------------------------
  Link options
---------------------------------------------------------------------------------*/
#define BLE_FEATURE true
#define WEB_FEATURE false
#define CLI_FEATURE true

/*---------------------------------------------------------------------------------
  
---------------------------------------------------------------------------------*/
// define the oximeter IC type, 
  #define OXI_NULL        0   // no oximeter hardware
  #define OXI_AFE4490     1   // TI IC, no internal LEDs, single power 
  #define OXI_MAX30102    2   // with 2 color LEDs, two power supply
//#define OXI_MAX30101    3   // with 3 color LEDs, two power supply
//#define OXI_MAXM86161   4   // with 3 color LEDs, single power

#define SPO2_TYPE        OXI_MAX30102   

// temperature sensor define
#define TEMP_SENSOR_MAX30325  true
#define TEMP_SENSOR_TMP117    false
#if (TEMP_SENSOR_MAX30325&&TEMP_SENSOR_TMP117)
  #error You must only enable at least one!
#endif 
/*---------------------------------------------------------------------------------
  Test Parameters
---------------------------------------------------------------------------------*/
#define TEST_PRINT      true    // show  more print out
#define SIM_BATTERY     true    // potentiometer A simulates the bettery voltage detction
#define SIM_TEMPERATURE false   // potentiometer B to simulate the temperature sensor
/*---------------------------------------------------------------------------------
  PIN number defined by ESP-WROOM-32 IO port number
---------------------------------------------------------------------------------*/
const uint8_t ADS1292_CS_PIN    = 13;
const uint8_t ADS1292_START_PIN = 14;
const uint8_t ADS1292_DRDY_PIN  = 26;
const uint8_t ADS1292_PWDN_PIN  = 27;
const uint8_t I2C_SDA_PIN       = 25;
const uint8_t I2C_SCL_PIN       = 22;
const uint8_t AFE4490_CS_PIN    = 21;   //this IO pin is not used if not usxing AFE4490
const uint8_t OXIMETER_INT_PIN  = 39; 
const uint8_t SPO2_START_PIN    = 4;    //high to power on SPO2 chip, low to power off
const uint8_t PUSH_BUTTON_PIN   = 0;
const uint8_t LED_PIN           = 2;
const uint8_t SENSOR_VP_PIN     = 36; 

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

extern int system_init_error;

// This file is "forcedInclude" for every C/C++ code (NOT .ino code).
// The setting is in c_cpp_properties.json
// but this forceInclude does not work.
// "${workspaceFolder}/firmware/firmware.h"
   
//#undef __APPLE__                // turn this on, if some error messages happen


/*---------------------------------------------------------------------------------
 public functions & classes
---------------------------------------------------------------------------------*/
/***********************
 * ads1292r.cpp (ECG)
 ***********************/

class ADS1292R
{
public:
  void      init(void);
  void      getData(void);
private:
  uint8_t * fillTxBuffer  (uint8_t peakvalue,uint8_t respirationRate);
  void      add_heart_rate_histogram(uint8_t hr);
  uint8_t   mask_register_bits(uint8_t address, uint8_t data_in);
};
extern  ADS1292R      ads1292r;
extern  void          ads1292r_interrupt_handler(void);
extern  portMUX_TYPE  ads1292rMux;
extern  bool          hrvDataReady  ;
extern  bool          histogramReady;
/***********************
 * oximeter_afe4490.cpp
 ***********************/
class AFE4490
{
  public:
    void    init    (void);
    void    getData (void);

  private:  
    void          writeData(uint8_t address, uint32_t data);
    unsigned long readData (uint8_t address);
};
extern class AFE4490  afe4490;
/***********************
 * firmware.ino
 ***********************/
extern uint8_t    ppg_heart_rate, old_ppg_heart_rate;
extern uint8_t    ecg_heart_rate, old_ecg_heart_rate;
extern uint8_t    spo2_percent,   old_spo2_percent;
extern uint8_t    ecg_lead_off;
/***********************
 * spo
 ***********************/
void clear_interrupt();
extern volatile bool spo2_interrupt_flag;
/***********************
 * spo2_max3010x.cpp
 ***********************/
void initMax3010xSpo2();
void handleMax3010xSpo2();
void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer,
                                            float *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid);
/***********************
 * for firmware.ino
 ***********************/
void initBLE();
void handleBLE();
void handleWebClient();
void initBasicOTA();
void handleOTA();
void initAcceleromter();
void handelAcceleromter();
void oximeter_interrupt_handler();

float   getTemperature();
boolean initTemperature();
extern volatile bool bleDeviceConnected;

/***********************
 * for BLE.cpp
 ***********************/
#define PPG_QUEUE_SIZE  100
#define PPG_QUEUE_LEN   (sizeof(uint16_t))

#define ECG_QUEUE_SIZE  100
#define ECG_QUEUE_LEN   (sizeof(uint16_t))

#endif //__PUBLIC_H__