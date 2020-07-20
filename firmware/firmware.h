#ifndef __PUBLIC_H__
#define __PUBLIC_H__

#include <Arduino.h>

/*---------------------------------------------------------------------------------
  Link options
---------------------------------------------------------------------------------*/
#define BLE_FEATURE false
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

#define SPO2_TYPE        OXI_NULL   

// this number need change according to hardware
// it = read_unblocked_IR_value() + tolerance range
#define HUMAN_BODY_PRESENT_IR_THRESHOLD   1000  //FIXME this number need define

// temperature sensor define
#define TEMP_SENSOR_MAX30325  false
#define TEMP_SENSOR_TMP117    true
#if (TEMP_SENSOR_MAX30325&&TEMP_SENSOR_TMP117)
  #error You must only enable at least one!
#endif 
/*---------------------------------------------------------------------------------
  Test Parameters
---------------------------------------------------------------------------------*/
#define TEST_PRINT      true    // show  more print out
#define SIM_BATTERY     true    // potentiometer A simulates the bettery voltage detction
#define SIM_TEMPERATURE true    // potentiometer B to simulate the temperature sensor
#define JOYTICK_TEST    true    // use joy stick y to simulate 
#define ECG_BLE_TEST    false   // send fake ecg data to BLE
#define SIM_PPG         true    // use joy stick x to simulate PPG
/*---------------------------------------------------------------------------------
  PIN number defined by ESP-WROOM-32 IO port number
---------------------------------------------------------------------------------*/
const uint8_t ADS1292_CS_PIN   = 13;
const uint8_t ADS1292_START_PIN= 14;
const uint8_t ADS1292_DRDY_PIN = 26;
const uint8_t ADS1292_PWDN_PIN = 27;

const uint8_t AFE4490_CS_PIN    = 21;   //this IO pin is not used if not usxing AFE4490
const uint8_t OXIMETER_INT_PIN  = 39; 
const uint8_t SPO2_START_PIN    = 4;    //high to power on SPO2 chip, low to power off
const uint8_t PUSH_BUTTON_PIN   = 0;
const uint8_t LED_PIN           = 2;
const uint8_t SENSOR_VP_PIN     = 36; 

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
void    ads1292r_interrupt_handler(void);
extern  portMUX_TYPE  ads1292rMux;
extern  bool          ecgBufferReady;
extern  bool          hrvDataReady  ;
extern  bool          histogramReady;
/***********************
 * ecg_resp.cpp
 ***********************/
void Filter_CurrentECG_sample (int16_t *CurrAqsSample, int16_t *FilteredOut);
void Filter_CurrentRESP_sample(int16_t  CurrAqsSample, int16_t *FiltOut);
void QRS_Algorithm_Interface  (int16_t  CurrSample);
void Calculate_RespRate       (int16_t  CurrSample,volatile uint8_t *respirationRate);

extern volatile uint16_t  QRS_Heart_Rate;
extern volatile uint8_t   npeakflag;
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
extern int32_t    heart_rate_from_oximeter;
extern uint8_t    SpO2Level;
extern bool       SpO2Ready;
/***********************
 * spo2.cpp
 ***********************/
class SPO2
{
  public:
    void init();
    void handleData();
    void simulateData();
    void clear_interrupt();
    void save_to_ppg_buffer(uint8_t i);
    
    volatile bool interrupt_flag;
  private:
    uint16_t  ppg_data_cnt;
};
extern class SPO2 spo2;
/***********************
 * spo2_max3010x.cpp
 ***********************/
void initMax3010xSpo2();
void handleMax3010xSpo2();
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

#endif //__PUBLIC_H__