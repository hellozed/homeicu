#ifndef __PUBLIC_H__
#define __PUBLIC_H__

// define the oximeter IC type, 

  #define OXI_AFE4490     1   // TI IC, no internal LEDs, single power 
  #define OXI_MAX30102    2   // with 2 color LEDs, two power supply
//#define OXI_MAX30101    3   // with 3 color LEDs, two power supply
//#define OXI_MAXM86161   4   // with 3 color LEDs, single power

#define SPO2_TYPE        OXI_MAX30102   

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
const int OXIMETER_INT_PIN  = 39; 
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
typedef struct ads1292r_Record
{
  signed long raw_ecg;
  signed long raw_resp;
  uint32_t    status_reg;
} ADS1292Data;

void    ads1292r_interrupt_handler(void);
extern  portMUX_TYPE  ads1292rMux;
extern  bool          ecgBufferReady;
extern  bool          hrvDataReady  ;
extern  bool          histogramReady;

class ADS1292R
{
public:
  void      init(void);
  void      getData(void);
  void      getTestData(void);
private:
  int       chip_select; 
  void      SendCommand   (uint8_t data_in);
  void      WriteRegister (uint8_t READ_WRITE_ADDRESS, uint8_t DATA);
  void      ReadToBuffer  (void);
  void      pin_high_time (int pin, uint32_t ms);
  void      pin_low_time  (int pin, uint32_t ms);
  uint8_t * fillTxBuffer  (uint8_t peakvalue,uint8_t respirationRate);
  void      add_heart_rate_histogram(uint8_t hr);

  uint8_t   SPI_ReadBuffer[10];
};

extern class ADS1292R  ads1292r;
/***********************
 * ecg_resp.cpp
 ***********************/
class ADS1292Process
{
  public:
    void ECG_FilterProcess(int16_t * WorkingBuff, int16_t * CoeffBuf, int16_t* FilterOut);
    void Filter_CurrentECG_sample(int16_t *CurrAqsSample, int16_t *FilteredOut);
    void Calculate_HeartRate(int16_t CurrSample,volatile uint8_t *Heart_rate, volatile uint8_t *peakflag);
    void QRS_process_buffer(volatile uint8_t *Heart_rate,volatile uint8_t *peakflag);
    void QRS_check_sample_crossing_threshold( uint16_t scaled_result,volatile uint8_t *Heart_rate,volatile uint8_t *peakflag);  
    void Resp_FilterProcess(int16_t * RESP_WorkingBuff, int16_t * CoeffBuf, int16_t* FilterOut);
    void Filter_CurrentRESP_sample(int16_t CurrAqsSample, int16_t * FiltOut);
    void Calculate_RespRate(int16_t CurrSample,volatile uint8_t *respirationRate);
    void Respiration_Rate_Detection(int16_t Resp_wave,volatile uint8_t *respirationRate);
};
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

extern class SPO2 spo2;

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
void initAcceleromter();
void handelAcceleromter();
void oximeter_interrupt_handler();

float   getTemperature();
boolean initTemperature();

#endif //__PUBLIC_H__