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
uint8_t* read_send_data(uint8_t peakvalue,uint8_t respirationRate);
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
#define TEMPERATURE_READ_INTERVAL     10000
 
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

uint8_t ecg_data_buff [20];
uint8_t resp_data_buff[2];
uint8_t ppg_data_buff [20];
uint8_t lead_flag = 0x04;
uint8_t data_len  = 20;
uint8_t Sp02;

int16_t ecg_wave_sample,  ecg_filterout;
int16_t res_wave_sample,  resp_filterout;

uint16_t ecg_stream_cnt = 0;
uint16_t ppg_stream_cnt = 0;
uint16_t ppg_wave_ir;

String strValue = "";

bool histogramReady     = false;
bool temperatureReady   = false;
bool SpO2_calc_done     = false;
bool ecgBufferReady     = false;
bool ppgBufferReady     = false;
bool hrvDataReady       = false;
bool batteryDataReady   = false;
char DataPacket[30];

uint8_t battery_percent = 100;
uint8_t bodyTemperature;

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
  timerAlarmWrite(timer, 1000000, true);  // 1 second

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

    #if SIM_BATTERY
    read_battery_value();
    #endif 
  
    //reading the battery with same interval as temperature sensor
    #if SIM_TEMPERATURE
    read_temperature_value(); 
    #endif 

    // Serial.printf("Battery: %03d  %03d \r", battery_percent, bodyTemperature);
  }  
}
/*---------------------------------------------------------------------------------
 battery level check
---------------------------------------------------------------------------------*/
void read_battery_value()
{
  static unsigned int   adcSum    = 0;
  static int            readCount = 0;
  unsigned int          adc;

  #define batteryAverageTimes 10
  
  adc = analogRead(SENSOR_VP_PIN);

  #if SIM_BATTERY
  // ESP32 ADC return a 12bit value 0~4095 (uint16_t) 
  // i = 0~4095 simulate y = 3600~4100
  // y = 3600 + (4100-3600)*i/4095
  adc = 3600 + (4100-3600)*adc/4095;
  #endif 
  
  adcSum += adc;

  if (readCount < batteryAverageTimes-1)
    readCount++;
  else
  { 
    adcSum = (adcSum / batteryAverageTimes);
    
    //Serial.printf("r:%05u adc:%05u %05u %d\r\n", 
    //analogRead(SENSOR_VP_PIN), adc, adcSum, battery_percent);

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

    if (adcSum >=4000)
      battery_percent = Percent(adcSum, 4100, 4000,100, 80);
    else if (adcSum >=3900)
      battery_percent = Percent(adcSum, 4000, 3900, 80, 60);
    else if (adcSum >=3800)
      battery_percent = Percent(adcSum, 3900, 3800, 60, 45);
    else if (adcSum >=3700)
      battery_percent = Percent(adcSum, 3800, 3700, 45, 30);
    else if (adcSum >=3600)
      battery_percent = Percent(adcSum, 3700, 3600, 30, 20);         
    else
      battery_percent = 10;

    readCount=0;
    adcSum=0;
    batteryDataReady = true; 
  }
}
 
void read_temperature_value()
{
  //if ((time_count++ * (1000/SAMPLING_RATE)) > TEMPERATURE_READ_INTERVAL)
  {      
    float temp;

    #if TEMP_SENSOR_MAX30325
    temp = tempSensor.getTemperature()*100; // read bodyTemperature for every 100ms
    bodyTemperature =  (uint16_t) temp;         // °C 
    #endif

    #if TEMP_SENSOR_TMP117
    // Data Ready is a flag for the conversion modes
    // the dataReady flag should always be high in continuous conversion 
    if (tempSensor.dataReady()) 
    {
      float tempC = tempSensor.readTempC();
      float tempF = tempSensor.readTempF();
      Serial.println(); // Create a white space for easier viewing
      Serial.print("Temperature in Celsius: ");
      Serial.println(tempC);
      Serial.print("Temperature in Fahrenheit: ");
      Serial.println(tempF);
      bodyTemperature = tempC;
    }
    #endif 

    #if SIM_TEMPERATURE
    temp =  (float)analogRead(SENSOR_TEMP);
    temp =  temp*100/4096;
    bodyTemperature = temp;
    #endif
  
    time_count = 0;
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
      DataPacket[14]    = 0;
      DataPacket[16]    = 0;
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
      
      if(ecg_stream_cnt >=18)
      {
        ecgBufferReady = true;
        ecg_stream_cnt = 0;
      }

      DataPacket[14] = RespirationRate;
      DataPacket[16] = heart_rate;
    }
  }

  memcpy(&DataPacket[0], &ecg_filterout, 2);
  memcpy(&DataPacket[2], &resp_filterout, 2);
  // SpO2 PPG 
  SPI.setDataMode (SPI_MODE0);
  afe4490.getData(&afe44xx_raw_data,AFE4490_CS_PIN,AFE4490_DRDY_PIN);
  ppg_wave_ir = (uint16_t)(afe44xx_raw_data.IR_data>>8);
  ppg_wave_ir = ppg_wave_ir;
  
  ppg_data_buff[ppg_stream_cnt++] = (uint8_t)ppg_wave_ir;
  ppg_data_buff[ppg_stream_cnt++] = (ppg_wave_ir>>8);

  if(ppg_stream_cnt >=18)
  {
    ppgBufferReady = true;
    ppg_stream_cnt = 0;
  }

    memcpy(&DataPacket[4], &afe44xx_raw_data.IR_data, sizeof(signed long));
    memcpy(&DataPacket[8], &afe44xx_raw_data.RED_data, sizeof(signed long));
  
  if( afe44xx_raw_data.buffer_count_overflow)
  {
    if (afe44xx_raw_data.spo2 == -999)
    {
      DataPacket[15] = 0;
      Sp02 = 0;
    }
    else
    {
      DataPacket[15] =  afe44xx_raw_data.spo2;
      Sp02 = (uint8_t)afe44xx_raw_data.spo2;       
    }
    SpO2_calc_done = true;
    afe44xx_raw_data.buffer_count_overflow = false;
  }

    DataPacket[17] = 80;  //bpsys
    DataPacket[18] = 120; //bp dia
    DataPacket[19]=  ads1292r_raw_data.status_reg;  

  SPI.setDataMode (SPI_MODE1);

  // temperature, battery
  read_temperature_value();
  read_battery_value();

}