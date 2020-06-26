/*---------------------------------------------------------------------------------
  Main code of the HomeICU project.

  Arduino IDE and VSCode is needed to build/download into boards. 

  Arduino Language Reference:
  https://www.arduino.cc/en/Reference/

  Heart rate and respiration computation based on original code from Texas Instruments
  requires a g4p_control graphing library for processing. 

  Downloaded from Processing IDE Sketch->Import Library->Add Library->G4P Install

  To view the build-out, please go to the build folder. 
---------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------
 Arduino/ESP32 library
---------------------------------------------------------------------------------*/
#include <SPI.h>
#include <Wire.h>         // I2C library
#include <ArduinoOTA.h>   // On-The-Air upload (wifi)
/*---------------------------------------------------------------------------------
 HomeICU driver code
---------------------------------------------------------------------------------*/
#include "ADS1292r.h"
#include "ecg_resp.h"
#include "AFE4490_Oximeter.h"
#include "spo2_algorithm.h"
#include "version.h"
#include "firmware.h"
#include "TMP117.h"
#include "MAX30205.h"

void initBLE();
void handleBLEstack();
void handleWebClient();
void setupWebServer();
void setupBasicOTA();
void add_heart_rate_histogram(uint8_t hr);
uint8_t *read_send_data(uint8_t peakvalue,uint8_t respirationRate);
void getTestData(uint8_t *p, int len);
/*---------------------------------------------------------------------------------
 Temperature sensor (ONLY turn on one of them)
---------------------------------------------------------------------------------*/
#define TEMP_SENSOR_MAX30325  false
#define TEMP_SENSOR_TMP117    false  

/*---------------------------------------------------------------------------------
  PIN number defined by ESP-WROOM-32 IO port number
---------------------------------------------------------------------------------*/
const int ADS1292_DRDY_PIN  = 26;
const int ADS1292_CS_PIN    = 13;
const int ADS1292_START_PIN = 14;
const int ADS1292_PWDN_PIN  = 27;
const int PUSH_BUTTON_PIN   = 0;
const int AFE4490_CS_PIN    = 21; 
const int AFE4490_DRDY_PIN  = 39; 
const int AFE4490_PWDN_PIN  = 4;
const int LED_PIN           = 2;
const int SENSOR_VP_PIN     = 36;   //GPIO36, ADC1_CH0

#if JOY_TEST
const int JOYX_PIN          = 39;   //GPIO39, ADC3_CH0
const int JOYY_PIN          = 34;   //GPIO34, ADC6_CH0
#endif

#if SIM_TEMPERATURE
const int SENSOR_TEMP       = 35;   //GPIO35, ADC6_CH0
#endif
/*---------------------------------------------------------------------------------
 constant and global variables
---------------------------------------------------------------------------------*/
 
volatile uint8_t  heart_rate = 0;
volatile uint8_t  HeartRate_prev = 0;
volatile uint8_t  RespirationRate=0;
volatile uint8_t  RespirationRate_prev = 0;
volatile uint8_t  npeakflag = 0;
volatile long     time_count=0;

volatile uint32_t buttonInterruptTime = 0;
volatile int      buttonEventPending = false;

volatile SemaphoreHandle_t timerSemaphore;
hw_timer_t * timer = NULL;

portMUX_TYPE buttonMux  = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE ads1292Mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux   = portMUX_INITIALIZER_UNLOCKED;


uint8_t ecg_data_buff[ECG_BUFFER_SIZE];
uint8_t resp_data_buff[2];  //FIXME add respiration rate send to ble
uint8_t ppg_data_buff[PPG_BUFFER_SIZE];
uint8_t lead_flag = 0x04;
uint8_t Sp02;

int16_t ecg_wave_sample,  ecg_filterout;
int16_t res_wave_sample,  resp_filterout;

uint16_t ecg_stream_cnt = 0;
uint16_t ppg_stream_cnt = 0;
uint16_t ppg_wave_ir;

bool histogramReady     = false;
bool temperatureReady   = false;
bool SpO2_calc_done     = false;
bool ecgBufferReady     = false;
bool ppgBufferReady     = false;
bool hrvDataReady       = false;
bool batteryDataReady   = false;

uint8_t battery_percent = 100;
float bodyTemperature;

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

ads1292r        ADS1292R;                      // define class ads1292r
ads1292r_processing ECG_RESPIRATION_ALGORITHM; // define class ecg_algorithm
AFE4490         afe4490;
spo2_algorithm  spo2;
ads1292r_data   ads1292r_raw_data;
afe44xx_data    afe44xx_raw_data;

#if TEMP_SENSOR_MAX30325
MAX30205        tempSensor;
#endif

#if TEMP_SENSOR_TMP117
TMP117          tempSensor; 
#endif

int system_init_error = 0;  
/*---------------------------------------------------------------------------------
Interrupt Routine should: 

    Keep it short
    Don't use delay ()
    Don't do serial prints
    Make variables shared with the main code volatile and protected by "critical sections"
    Don't try to turn interrupts off or on

ESP32 has 2 cores, each has 6 internal peripheral interrupts:
3 x timer comparators
1 x performance monitor
3 x software interrupts.

Arduino normally use "NoInterrupts" and "Interrupts" to enable/disable interrupt
for critical sections. But "NoInterrupts" and "Interrupts" have not been 
implemented in ESP32 Arduino, please use the following methods.

portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

portENTER_CRITICAL(&myMutex);
//critical section
portEXIT_CRITICAL(&myMutex);

portENTER_CRITICAL_ISR(&myMutex);
//ISR section
portEXIT_CRITICAL_ISR(&myMutex);

use semaphore if using FreeRtos. Refer to:

http://www.iotsharing.com/2017/06/how-to-use-binary-semaphore-mutex-counting-semaphore-resource-management.html
http://www.gammon.com.au/interrupts
---------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------
  Button interrupt handler
---------------------------------------------------------------------------------*/
void IRAM_ATTR button_interrupt_handler()
{
  portENTER_CRITICAL_ISR(&buttonMux);
    buttonEventPending++;
    buttonInterruptTime = millis();
  portEXIT_CRITICAL_ISR(&buttonMux);
}

// if button was pressed
void doButton() {
  if (buttonEventPending>0)
  {
    if ((millis() - buttonInterruptTime) > 100)    //after a delay
    {
      if (digitalRead(PUSH_BUTTON_PIN) == 0)  //still LOW    
      {
        Serial.println("BUTTON Yes");
      }
      portENTER_CRITICAL(&buttonMux);
        buttonEventPending--;                 //clear the event pending flag
      portEXIT_CRITICAL(&buttonMux);
    } 
  } 
}  
/*---------------------------------------------------------------------------------
 Repeat Timer interrupt

 Note: code for killing the timer:
  if (timer) {
      // Stop and free timer
      timerEnd(timer);
      timer = NULL;
  }
---------------------------------------------------------------------------------*/
void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&timerMux);
  // Do something here
  portEXIT_CRITICAL_ISR(&timerMux);

  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void setupTimer() {
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every x microseconds).
  // Repeat the alarm (third parameter)

  #define TIMER_MICRO_SECONDS 1000000*0.2 // * second
  timerAlarmWrite(timer, TIMER_MICRO_SECONDS, true); 

  // Start an alarm
  timerAlarmEnable(timer);
}

void doTimer() 
{
  // if Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);

    portEXIT_CRITICAL (&timerMux);

    #if JOY_TEST
    // ESP32 ADC return a 12bit value 0~4095 (uint16_t) 
    {
    int x, y;
    x = analogRead(JOYX_PIN);
    y = analogRead(JOYY_PIN);
    //Serial.printf("%05d %05d %03d\r", x,y);
    }
    #endif 


    #if ECG_BLE_TEST

    // TIMER_MICRO_SECONDS
    // ECG_SAMPLE_RATE

    if (ecgBufferReady == false )     // refill the data
      getTestData(ecg_data_buff, ECG_BUFFER_SIZE); // 125 Sample rate, 0.2s
      ecgBufferReady = true; 
    #endif 
  }  
}
/*---------------------------------------------------------------------------------
 battery level check
---------------------------------------------------------------------------------*/
void read_battery_value()
{
  #define BAT_SAMPLE  5     
  
  static    uint16_t    adcSample[BAT_SAMPLE];
  static unsigned long  LastReadTime = 0;

  unsigned int          adc = 0;
  int i;

  #define BATTERY_READ_INTERVAL     (0.2*1000)   //millis
  
  // check if the right time to read
  if (millis() - LastReadTime < BATTERY_READ_INTERVAL)
    return;   // not the right time

  // it is the right time, continue read adc
  LastReadTime = millis();    

  // average several adc sample to reduce noise
  
  // shift adc sample into buffer      
  for (i=BAT_SAMPLE-1; i>0; i--)
    adcSample[i] = adcSample[i-1];
  adcSample[0] = analogRead(SENSOR_VP_PIN);     // FIXME may use ADC IC to do so

  // sum sample data together   
  for (i=0; i<BAT_SAMPLE; i++)
    adc += adcSample[i];

  // divide it by sample number  
  adc = adc/BAT_SAMPLE;

  #if SIM_BATTERY
  // ESP32 ADC return a 12bit value 0~4095 (uint16_t) 
  // i = 0~4095 simulate y = 3600~4100
  // y = 3600 + (4100-3600)*i/4095
  adc = 3600 + (4100-3600)*adc/4095;
  #endif 
  
  /*
  ADC value (3600~4100) -> Battery Life (20%~100%)
  It is not linear relationship between ADC and battery.
  Adc : Batterry 
  4100 100%
  4000 80%
  3900 60%
  3800 45%
  3700 30%
  3600 20%
  Batter Life = x Hours
  */
  #define Percent(adcNow, adcH, adcL, percentH, percentL) \
          (percentL+(percentH-percentL)*(adcNow-adcL)/(adcH-adcL))

  if (adc >=4000)
    battery_percent = Percent(adc, 4100, 4000,100, 80);
  else if (adc >=3900)
    battery_percent = Percent(adc, 4000, 3900, 80, 60);
  else if (adc >=3800)
    battery_percent = Percent(adc, 3900, 3800, 60, 45);
  else if (adc >=3700)
    battery_percent = Percent(adc, 3800, 3700, 45, 30);
  else if (adc >=3600)
    battery_percent = Percent(adc, 3700, 3600, 30, 20);         
  else
    battery_percent = 10;

  //Serial.printf("adc:%u average+adjust %u battery: %u%% \r", 
  //              adcSample[0], adc, battery_percent);
  batteryDataReady = true; 

}
 
void read_temperature_value()
{
  #define TEMPERATURE_READ_INTERVAL     (2*1000)  //millis
  
  static unsigned long LastReadTime = 0;

  if (millis() - LastReadTime > TEMPERATURE_READ_INTERVAL)
  { 
    LastReadTime = millis();     

    #if TEMP_SENSOR_MAX30325
    bodyTemperature = tempSensor.getTemperature()*100;    // °C 
    #endif

    #if TEMP_SENSOR_TMP117
    // Data Ready is a flag for the conversion modes
    // the dataReady flag should always be high in continuous conversion 
    if (tempSensor.dataReady()) 
      bodyTemperature = tempSensor.readTempC();;
    #endif 
    
    #if SIM_TEMPERATURE
    bodyTemperature =  (float)analogRead(SENSOR_TEMP)*100/4096;
    #endif
  
    temperatureReady = true;
  }
}
/*---------------------------------------------------------------------------------
 The ESP32 has four SPI buses, only two of them are available to use, HSPI and VSPI. 
 Simply using the SPI API as illustrated in Arduino examples will use VSPI, leaving HSPI unused.
 
 If we simply initialize two instances of the SPI class for both
 of these buses both can be used. When using these the Arduino
 way only will be outputting at a time.

 SPI	  MOSI	  MISO	  CLK	    CS
 VSPI	GPIO23	GPIO19	GPIO18	GPIO5
 HSPI	GPIO13	GPIO12	GPIO14	GPIO15
 FIXME This design only uses VSPI, the default CS pin is IO5, but this design use IO21.
 IDE provides another example of the SPI code.
---------------------------------------------------------------------------------*/
void initSPI()
{
  SPI.begin();
  Wire.begin(25,22);  //FIXME test this line
  SPI.setClockDivider (SPI_CLOCK_DIV16);
  SPI.setBitOrder     (MSBFIRST);
  SPI.setDataMode     (SPI_MODE0);
  delay(10);        //delay 10ms
  afe4490.Init (AFE4490_CS_PIN,AFE4490_PWDN_PIN);
  delay(10); 
  SPI.setDataMode (SPI_MODE1);          //Set SPI mode as 1
  delay(10);
}
/*---------------------------------------------------------------------------------
The setup() function is called when a sketch starts. Use it to initialize variables, 
pin modes, start using libraries, etc. The setup() function will only run once, 
after power-up or reset of the  board.
---------------------------------------------------------------------------------*/
void setup()
{
  uint64_t chipid;
  
  // Make sure serial port on first
  // Setup serial port U0UXD for programming and reset/boot
  Serial.begin  (115200);   // Baudrate for serial communication
  chipid=ESP.getEfuseMac(); // chip ID is MAC address(6 bytes).
  
  Serial.println("************************************************");
  Serial.println("HomeICU Firmware");
  Serial.println("************************************************");
  Serial.printf ("Version: %s Git commits: %s \r\n", homeicu_version, homeicu_commits);
	Serial.printf ("MAC address: %04X %08X \r\n",(uint16_t)(chipid>>32), (uint32_t)chipid);

  // initialize the data ready and chip select pins:
  // Pin numbers are defined as ESP-WROOM-32, not as ESP32 processor
  pinMode(ADS1292_DRDY_PIN,   INPUT);  
  pinMode(ADS1292_CS_PIN,     OUTPUT);    
  pinMode(ADS1292_START_PIN,  OUTPUT);
  pinMode(ADS1292_PWDN_PIN,   OUTPUT);  
  pinMode(LED_PIN,            OUTPUT); 
  pinMode(AFE4490_PWDN_PIN,   OUTPUT);
  pinMode(AFE4490_CS_PIN,     OUTPUT);  // slave select
  pinMode(AFE4490_DRDY_PIN,   INPUT);   // data ready 
  pinMode(PUSH_BUTTON_PIN,    INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_PIN), button_interrupt_handler, FALLING);
 
  setupTimer();               

  initBLE();                  //low energy blue tooth 
   
  initSPI();                  //initialize SPI

  setupBasicOTA();            //Over The Air for code uploading
  #if WEB_UPDATE
  setupWebServer();           //Web server for code uploading
  #endif 

  //initialize ADS1292 slave
  ADS1292R.Init(ADS1292_CS_PIN,ADS1292_PWDN_PIN,ADS1292_START_PIN);  
   
  delay(10); 
  
  // Digital2 is attached to Data ready pin of AFE is interrupt0 in ARduino
  attachInterrupt(digitalPinToInterrupt(ADS1292_DRDY_PIN),ads1292r_interrupt_handler, FALLING ); 
  
  #if (TEMP_SENSOR_MAX30325 | TEMP_SENSOR_TMP117)
  if (tempSensor.begin()) 
    Serial.println("Temperature sensor: OK.");
  else
    Serial.println("Temperature sensor: Missing.");
  #endif 

  Serial.printf("Setup() done. Error number = %d\r\n\r\n",system_init_error);
}
/*---------------------------------------------------------------------------------
After creating a setup() function, which initializes and sets the initial values, 
the loop() function loops consecutively, allowing your program to change and respond.
---------------------------------------------------------------------------------*/
void loop()
{
  boolean result;

  doTimer();                  // process timer event
  doButton();                 // process button event
  ArduinoOTA.handle();        // "On The Air" update function 

  #if WEB_UPDATE
  handleWebClient();          // web server
  #endif 

  handleBLEstack();           // handle bluetooth low energy

  // handle ADS1292/ECG/RESP
#if 0  //FIXME
  result = ADS1292R.getData(ADS1292_DRDY_PIN,ADS1292_CS_PIN,&ads1292r_raw_data);
  if (result == true)
  {  
    // ignore the lower 8 bits out of 24bits 
    ecg_wave_sample = (int16_t)(ads1292r_raw_data.raw_ecg  >> 8);  
    res_wave_sample = (int16_t)(ads1292r_raw_data.raw_resp >> 8);
  
    if (!((ads1292r_raw_data.status_reg & 0x1f) == 0))
    { // measure lead is OFF the body 
      lead_flag         = 0x04;
      ecg_filterout     = 0;
      resp_filterout    = 0;      
    }  
    else
    { // the measure lead is ON the body 
      lead_flag         = 0x06;
      // filter out the line noise @40Hz cutoff 161 order
      ECG_RESPIRATION_ALGORITHM.Filter_CurrentECG_sample  (&ecg_wave_sample,&ecg_filterout);   
      ECG_RESPIRATION_ALGORITHM.Calculate_HeartRate       (ecg_filterout,   &heart_rate,&npeakflag); 
      ECG_RESPIRATION_ALGORITHM.Filter_CurrentRESP_sample (res_wave_sample, &resp_filterout);
      ECG_RESPIRATION_ALGORITHM.Calculate_RespRate        (resp_filterout,  &RespirationRate);   
      if(npeakflag == 1)
      {
        read_send_data(heart_rate,RespirationRate);
        add_heart_rate_histogram(heart_rate);
        npeakflag = 0;
      }
   
      ecg_data_buff[ecg_stream_cnt++] = (uint8_t)ecg_wave_sample; //ecg_filterout;
      ecg_data_buff[ecg_stream_cnt++] = (ecg_wave_sample>>8);     //ecg_filterout>>8;
      
      if(ecg_stream_cnt >=ECG_BUFFER_SIZE)
      {
        ecgBufferReady = true;
        ecg_stream_cnt = 0;
      }
    }
  }

  // SpO2 PPG 
  SPI.setDataMode (SPI_MODE0);
  afe4490.getData(&afe44xx_raw_data,AFE4490_CS_PIN,AFE4490_DRDY_PIN);
  ppg_wave_ir = (uint16_t)(afe44xx_raw_data.IR_data>>8);
  ppg_wave_ir = ppg_wave_ir;
  
  ppg_data_buff[ppg_stream_cnt++] = (uint8_t)ppg_wave_ir;
  ppg_data_buff[ppg_stream_cnt++] = (ppg_wave_ir>>8);

  if(ppg_stream_cnt >=PPG_BUFFER_SIZE)
  {
    ppgBufferReady = true;
    ppg_stream_cnt = 0;
  }

  if( afe44xx_raw_data.buffer_count_overflow)
  {
    if (afe44xx_raw_data.spo2 == -999)
    {
      Sp02 = 0;
    }
    else
    {
      Sp02 = (uint8_t)afe44xx_raw_data.spo2;       
    }
    SpO2_calc_done = true;
    afe44xx_raw_data.buffer_count_overflow = false;
  }

  SPI.setDataMode (SPI_MODE1);
#endif //0 //FIXME
  // temperature, battery
  read_temperature_value();
  read_battery_value();

}